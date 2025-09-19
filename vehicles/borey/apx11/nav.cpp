#include <apx.h>

constexpr const uint16_t TASK_MAIN_MS{200}; //msec
constexpr const uint16_t TASK_ERS_MS{100};  //msec

constexpr const port_id_t PORT_ID_ESC{50};

struct ESC_VCP_Data
{
    uint8_t temp;
    float voltage;
    float current;
    uint16_t consumption;
    uint16_t rpm;
};

ESC_VCP_Data esc_data;

const uint8_t PACK_SIZE_ESC = 10;
uint8_t esc_tbuf[PACK_SIZE_ESC] = {};

using m_eng_temp = Mandala<mandala::sns::env::eng::temp>;
using m_eng_volt = Mandala<mandala::sns::env::eng::voltage>;
using m_eng_rpm = Mandala<mandala::sns::env::eng::rpm>;

using m_air = Mandala<mandala::est::env::usrb::b8>;

using m_launch = Mandala<mandala::ctr::env::ers::launch>; //launch
using m_cmd_cut = Mandala<mandala::cmd::nav::eng::cut>;   //eng cut
using m_release = Mandala<mandala::ctr::env::ers::rel>;   //release

//datalink
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;

using m_airspeed = Mandala<mandala::est::nav::air::airspeed>;
using m_altitude = Mandala<mandala::est::nav::pos::altitude>;
using m_vspeed = Mandala<mandala::est::nav::pos::vspeed>;

using m_ax = Mandala<mandala::est::nav::acc::x>;
using m_ay = Mandala<mandala::est::nav::acc::y>;
using m_az = Mandala<mandala::est::nav::acc::z>;

static constexpr const float AIR_ALT{150.f}; //[m]
static constexpr const float AIR_SPD{15.f};  //[m/sec]

//parachute release
static constexpr const float GRAVITY_NORM{9.8f};   //[m/sec^2]
static constexpr const float RELEASE_VDOWN{-0.5f}; //[m/sec]
static constexpr const float RELEASE_ALT{10.f};    //[m]
static constexpr const float G_MAX{5.5f};          //[G]

uint32_t g_LastVspeedReleaseTime{0};

bool g_checkAirLockout{false};
bool g_onReleaseLockout{false};

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    schedule_periodic(task("on_ers"), TASK_ERS_MS);

    receive(PORT_ID_ESC, "esc_handler");

    //datalink
    m_ltt();
    m_health();
    m_mode();

    m_air();

    m_airspeed();
    m_altitude();
    m_vspeed();
    m_ax();
    m_ay();
    m_az();

    m_air::publish(0u);

    m_launch("on_launch"); // subscribe

    return 0;
}

EXPORT void on_main()
{
    //datalink
    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning) {
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }

    //save data to mandala
    m_eng_temp::publish((uint32_t) esc_data.temp);
    m_eng_volt::publish((float) esc_data.voltage);
    m_eng_rpm::publish((uint32_t) esc_data.rpm);

    //printf("temp:%u", esc_data.temp);
    //printf("voltage:%.2f", esc_data.voltage);
    //printf("current:%.2f", esc_data.current);
    //printf("consumption:%u", esc_data.consumption);
    //printf("rpm:%u", esc_data.rpm);
}

bool checkVSpeedAndAltitudeRelease()
{
    const float vspeed = m_vspeed::value();
    const float altitude = m_altitude::value();
    const uint32_t now = time_ms();

    if ((vspeed > RELEASE_VDOWN) && (altitude < RELEASE_ALT)) {
        return (now - g_LastVspeedReleaseTime > 500);
    }
    g_LastVspeedReleaseTime = now;

    return false;
}

bool checkGForceRelease()
{
    float altitude = (float) m_altitude::value();

    float Ax = (float) m_ax::value() / GRAVITY_NORM;
    float Ay = (float) m_ay::value() / GRAVITY_NORM;
    float Az = (float) m_az::value() / GRAVITY_NORM;

    float val = sqrt(Ax * Ax + Ay * Ay + Az * Az);

    if (altitude < RELEASE_ALT && val > G_MAX) {
        printf("VM:G_VAL %f\n", val);
        return true;
    }

    return false;
}

EXPORT void on_ers()
{
    const float altitude = (float) m_altitude::value();
    const float airspeed = (float) m_airspeed::value();

    //check air state
    if (!g_checkAirLockout && altitude > AIR_ALT && airspeed > AIR_SPD) {
        g_checkAirLockout = true;
        m_air::publish(1u);
        printf("VM:Start AIR\n");
    }

    //release parachute
    const bool m_air_val = (bool) m_air::value() || g_checkAirLockout;
    const bool lvs_release = checkVSpeedAndAltitudeRelease();
    const bool gmax_release = checkGForceRelease();
    if (m_air_val && !g_onReleaseLockout && (altitude < RELEASE_ALT) && (lvs_release || gmax_release)) {
        g_onReleaseLockout = true;
        m_release::publish(1u);
        printf("VM:REL ok\n");
        printf("VM:LVS %u\n", lvs_release);
        printf("VM:GMAX %u\n", gmax_release);
    }
}

uint8_t update_crc8(uint8_t data, uint8_t crc)
{
    data ^= crc;

    for (uint8_t i = 0; i < 8; i++) {
        data = uint8_t((data & 0x80) ? 0x07 ^ (data << 1) : (data << 1));
    }
    return data;
}

uint8_t get_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc{0};
    for (uint8_t i = 0; i < len; i++) {
        crc = update_crc8(data[i], crc);
    }
    return crc & 0xFF;
}

EXPORT void esc_handler(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_ESC) {
        return;
    }

    memcpy(esc_tbuf, data, size);

    if (get_crc8(esc_tbuf, PACK_SIZE_ESC - 1) != esc_tbuf[PACK_SIZE_ESC - 1]) {
        return;
    }

    esc_data.temp = data[0];
    esc_data.voltage = float((esc_tbuf[1] << 8) | (esc_tbuf[2])) / 100.f;
    esc_data.current = float((esc_tbuf[3] << 8) | (esc_tbuf[4])) / 100.f;
    esc_data.consumption = uint16_t((esc_tbuf[5] << 8) | (esc_tbuf[6]));
    esc_data.rpm = uint16_t((esc_tbuf[7] << 8) | (esc_tbuf[8])) * 100 / 7u;
}

EXPORT void on_launch()
{
    if (m_launch::value() == 1u) {
        sleep(100);
        m_cmd_cut::publish(1u);
    }
}
