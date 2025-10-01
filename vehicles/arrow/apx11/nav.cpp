//Fuel control protocol
//UART: 19200 8 N 1

#include <apx.h>

//#define EQUAL_TANKS

static constexpr const port_id_t port_fuel_id{11};

const uint16_t TASK_MAIN_MS{100}; //msec
const uint16_t TASK_ERS_MS{100};  //msec
const uint16_t TASK_FUEL_MS{500}; //msec

//FUEL
const uint8_t ADR_FUEL_SENS1{75};
const uint8_t ADR_FUEL_SENS2{76};
const uint8_t ADR_FUEL_SENS3{77};

const uint8_t MSG_FUEL_SIZE{9}; //FUEL

#ifdef EQUAL_TANKS
const float V_MAX1{16.7f}; //liters
const float V_MAX2{16.7f}; //liters
const float V_MAX3{16.7f}; //liters

const float CRITICAL_LOW{7.0f};  //7% = 1.1l
const float TANK_1_POINT{13.0f}; //13% = 2l
const float TANK_2_POINT{25.0f}; //25% = 3.9l
const float TANK_3_POINT{51.0f}; //51% = 8l
#else
const float V_MAX1{17.0f}; //liters
const float V_MAX2{18.3f}; //liters
const float V_MAX3{24.6f}; //liters

const float CRITICAL_LOW{7.0f};  //7% = 1.1l
const float TANK_1_POINT{20.0f}; //20% = 3.4l
const float TANK_2_POINT{25.0f}; //25% = 4.5l
const float TANK_3_POINT{60.0f}; //60% = 14.7l
#endif

uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};

uint8_t ADR_FUEL[3] = {ADR_FUEL_SENS1, ADR_FUEL_SENS2, ADR_FUEL_SENS3};
uint8_t idx_fuel_array = 0;

uint32_t startTimerPumpON;

uint8_t pump_stage{1}; //default stage

const uint8_t TIME_SA{5};     //sec
const uint8_t DELAY_PUMP{15}; //sec between different pumps

bool fuel_mc_old{false};
bool ignition_old{false};

struct _fuel
{
    _fuel(float v_max)
        : V_MAX(v_max)
        , liters(0.f)
        , percent(0.f)
        , time_ans(0)
    {}

    void set_percent(float p)
    {
        percent = p;
        liters = (p * V_MAX) / 100.f;
        time_ans = time_ms();
    }

    float V_MAX;   // max tank liters
    float liters;  // liters
    float percent; // percent
    uint32_t time_ans;
};

_fuel fuel[3] = {_fuel(V_MAX1), _fuel(V_MAX2), _fuel(V_MAX3)};

//Engine
using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_eng_cut = Mandala<mandala::cmd::nav::eng::cut>;

//fuel
using m_fuel1 = Mandala<mandala::est::env::usr::u5>;
using m_fuel2 = Mandala<mandala::est::env::usr::u6>;
using m_fuel3 = Mandala<mandala::est::env::usr::u7>;

using m_fuel_p = Mandala<mandala::sns::env::fuel::level>;
using m_fuel_l = Mandala<mandala::est::env::usr::u8>;

using m_pump1 = Mandala<mandala::ctr::env::sw::sw5>;
using m_pump2 = Mandala<mandala::ctr::env::sw::sw6>;
using m_pump3 = Mandala<mandala::ctr::env::sw::sw7>;

using m_warn1 = Mandala<mandala::est::env::usrb::b5>;
using m_warn2 = Mandala<mandala::est::env::usrb::b6>;
using m_warn3 = Mandala<mandala::est::env::usrb::b7>;

using m_fuel_mc = Mandala<mandala::ctr::env::sw::sw8>; //manual control

//datalink
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;
using m_stage = Mandala<mandala::cmd::nav::proc::stage>;

//ers
using m_ers1 = Mandala<mandala::ctr::env::ers::launch>;
using m_ers2 = Mandala<mandala::est::env::usrb::b1>;
using m_ers3 = Mandala<mandala::ctr::env::ers::rel>;

using m_air = Mandala<mandala::est::env::usrb::b8>;

using m_roll = Mandala<mandala::est::nav::att::roll>;
using m_pitch = Mandala<mandala::est::nav::att::pitch>;
using m_airspeed = Mandala<mandala::est::nav::air::airspeed>;
using m_altitude = Mandala<mandala::est::nav::pos::altitude>;
using m_vspeed = Mandala<mandala::est::nav::pos::vspeed>;

using m_ax = Mandala<mandala::est::nav::acc::x>;
using m_ay = Mandala<mandala::est::nav::acc::y>;
using m_az = Mandala<mandala::est::nav::acc::z>;

//emergency parachute deployment
static constexpr const float ERS_VDOWN{-30.f}; //[m/sec]
static constexpr const float ERS_ROLL{70.f};   //[dec]
static constexpr const float ERS_PITCH{50.f};  //[dec]

//parachute release
static constexpr const float GRAVITY_NORM{9.8f};   //[m/sec^2]
static constexpr const float RELEASE_VDOWN{-0.5f}; //[m/sec]
static constexpr const float G_MAX{5.5f};          //[G]

static constexpr const float AIR_ALT{500.f};  //[m]
static constexpr const float AIR_SPD{20.f};   //[m/sec]
static constexpr const float ERS2_ALT{200.f}; //[m]
static constexpr const float ERS3_ALT{10.f};  //[m]

uint16_t WAIT_TIME{3000}; //[msec]
bool g_onERS{false};      //flag that ERS1 was activated

uint32_t g_lastRollPitchTime{0};
uint32_t g_lastVspeedTime{0};
uint32_t g_LastVspeedReleaseTime{0};
uint32_t g_LastMaxRollInTakeoffModeTime{0};

bool g_checkAirLockout{false};
bool g_lowAltLockout{false};
bool g_AttAndVSpeedLockout{false};
bool g_onERS2Lockout{false};
bool g_onERS3Lockout{false};

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    schedule_periodic(task("on_ers"), TASK_ERS_MS);
    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    receive(port_fuel_id, "on_fuel_serial");

    //fuel
    m_fuel_p();
    m_fuel_l();

    m_fuel1();
    m_fuel2();
    m_fuel3();

    m_pump1();
    m_pump2();
    m_pump3();

    m_warn1();
    m_warn2();
    m_warn3();

    m_fuel_mc();

    //datalink
    m_ltt();
    m_health();
    m_mode();
    m_stage();

    //ers
    m_ers1();
    m_ers2();
    m_ers3();

    m_pwr_eng();
    m_air();

    m_roll();
    m_pitch();
    m_airspeed();
    m_altitude();
    m_vspeed();
    m_ax();
    m_ay();
    m_az();

    uint32_t now = time_ms();

    g_lastRollPitchTime = now;
    g_lastVspeedTime = now;
    g_LastVspeedReleaseTime = now;

    m_ers1::publish(false);
    m_ers2::publish(false);
    m_ers3::publish(false);

    m_air::publish(false);

    m_ers1("on_launch"); // subscribe

    return 0;
}

EXPORT void on_main()
{
    //datalink
    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning
        && (uint32_t) m_mode::value() != mandala::proc_mode_TAXI) {
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }
}

bool checkRollAndPitch()
{
    const float roll = fabs(m_roll::value() * R2D);
    const float pitch = fabs(m_pitch::value() * R2D);
    const uint32_t now = time_ms();

    if (roll > ERS_ROLL || pitch > ERS_PITCH) {
        return (now - g_lastRollPitchTime > 2000);
    }
    g_lastRollPitchTime = now;

    return false;
}

bool checkVSpeed()
{
    const float vspeed = m_vspeed::value();
    const uint32_t now = time_ms();

    if (vspeed < ERS_VDOWN) {
        return (now - g_lastVspeedTime > 1000);
    }
    g_lastVspeedTime = now;

    return false;
}

bool checkTakeOFFSafety()
{
    const bool mode = (uint8_t) m_mode::value() == mandala::proc_mode_TAKEOFF;
    const bool roll = fabs(m_roll::value() * R2D) > ERS_ROLL;
    const bool vspeed = m_vspeed::value() < -1.f;
    const uint32_t now = time_ms();

    if (mode && roll && vspeed) {
        return (now - g_LastMaxRollInTakeoffModeTime > 1000);
    }
    g_LastMaxRollInTakeoffModeTime = now;

    return false;
}

bool checkVSpeedAndAltitudeRelease()
{
    const float vspeed = m_vspeed::value();
    const float altitude = m_altitude::value();
    const uint32_t now = time_ms();

    if ((vspeed > RELEASE_VDOWN) && (altitude < ERS3_ALT)) {
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

    if (altitude < ERS3_ALT && val > G_MAX) {
        printf("VM:G_VAL %f\n", val);
        return true;
    }

    return false;
}

EXPORT void on_ers()
{
    const float altitude = (float) m_altitude::value();
    const float airspeed = (float) m_airspeed::value();
    const bool ers1 = (bool) m_ers1::value();

    //check air state
    if (!g_checkAirLockout && altitude > AIR_ALT && airspeed > AIR_SPD) {
        g_checkAirLockout = true;
        m_air::publish(true);
        printf("VM:Start AIR\n");
    }

    //minimum safe altitude for parachute release
    //if (g_checkAirLockout && !g_lowAltLockout && altitude < ERS2_ALT) {
    //    g_lowAltLockout = true;
    //    g_onERS = true;
    //    WAIT_TIME = 100;
    //    m_ers1::publish(1u);
    //    printf("VM:Low alt. ERS on\n");
    //}

    const bool m_air_val = (bool) m_air::value() || g_checkAirLockout;
    const bool is_rp = checkRollAndPitch();
    const bool is_vspd = checkVSpeed();

    const bool ers = (m_air_val && !g_AttAndVSpeedLockout && (is_rp || is_vspd)) || checkTakeOFFSafety();

    if (ers) {
        g_AttAndVSpeedLockout = true;
        g_onERS = true;
        m_ers1::publish(true);
        if (altitude < ERS2_ALT) {
            WAIT_TIME = 100;
        }

        printf("VM:ERS1 on\n");
        printf("VM:rp %d\n", is_rp);
        printf("VM:vspd %d\n", is_vspd);
    }

    if (ers1 || g_onERS) {
        g_onERS = true;
        sleep(WAIT_TIME);
        WAIT_TIME = 0;

        //main parachute
        if (!g_onERS2Lockout && altitude < ERS2_ALT) {
            g_onERS2Lockout = true;
            m_ers2::publish(true);
            printf("VM:ERS2 on\n");
        }

        //release parachute
        const bool lvs_release = checkVSpeedAndAltitudeRelease();
        const bool gmax_release = checkGForceRelease();
        if (g_onERS2Lockout && !g_onERS3Lockout && (lvs_release || gmax_release) && (altitude < ERS3_ALT)) {
            g_onERS3Lockout = true;
            m_ers3::publish(true);
            printf("VM:ERS3 ok\n");
            printf("VM:LVS %u\n", lvs_release);
            printf("VM:GMAX %u\n", gmax_release);
        }
    }
}

EXPORT void on_launch()
{
    if ((bool) m_ers1::value()) {
        sleep(100);
        m_eng_cut::publish(true);
        m_pwr_eng::publish(false);
    }
}

EXPORT uint8_t calcCRC(const uint8_t *buf, size_t size)
{
    uint8_t crc = 0x00;

    for (uint8_t j = 0; j < size; j++) {
        uint8_t i = buf[j] ^ crc;
        crc = 0;
        if (i & 0x01)
            crc ^= 0x5e;
        if (i & 0x02)
            crc ^= 0xbc;
        if (i & 0x04)
            crc ^= 0x61;
        if (i & 0x08)
            crc ^= 0xc2;
        if (i & 0x10)
            crc ^= 0x9d;
        if (i & 0x20)
            crc ^= 0x23;
        if (i & 0x40)
            crc ^= 0x46;
        if (i & 0x80)
            crc ^= 0x8c;
    }
    return crc % 256;
}

void turn_off_all_pumps()
{
    m_pump1::publish(false);
    m_pump2::publish(false);
    m_pump3::publish(false);
}

void turn_on_all_pumps()
{
    m_pump1::publish(true);
    m_pump2::publish(true);
    m_pump3::publish(true);
}

void turn_on_pump_1()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump2::value() || (bool) m_pump3::value()) {
            m_pump2::publish(false);
            m_pump3::publish(false);
            return;
        }
        if (!(bool) m_pump1::value()) {
            m_pump1::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_2()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value() || (bool) m_pump3::value()) {
            m_pump1::publish(false);
            m_pump3::publish(false);
            return;
        }
        if (!(bool) m_pump2::value()) {
            m_pump2::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_3()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value() || (bool) m_pump2::value()) {
            m_pump1::publish(false);
            m_pump2::publish(false);
            return;
        }
        if (!(bool) m_pump3::value()) {
            m_pump3::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void pump_stage_1() //get fuel from tank 3 until the point
{
    if (fuel[0].percent > TANK_3_POINT) {
        turn_on_pump_3();
    } else {
        pump_stage = 2;
        printf("fuel stage: 2");
    }
}

void pump_stage_2() //get fuel from tank 3 until empty and from 1 until the point
{
    if (fuel[0].percent > TANK_1_POINT) {
        if (fuel[2].percent > CRITICAL_LOW) {
            if (fuel[0].percent > (fuel[2].percent + 100 - TANK_3_POINT)) {
                turn_on_pump_1();
            } else {
                turn_on_pump_3();
            }
        } else {
            turn_on_pump_1();
        }
    } else {
        pump_stage = 3;
        printf("fuel stage: 3");
    }
}

void pump_stage_3() //get fuel from tank 2 until specified points
{
    if (fuel[2].percent < CRITICAL_LOW) {
        if (fuel[1].percent > TANK_2_POINT) {
            turn_on_pump_2();
        } else {
            pump_stage = 4;
            printf("fuel stage: 4");
        }
    } else {
        turn_on_pump_3();
    }
}

void pump_stage_4() //get fuel from tank 1 and 2 until empty
{
    if (fuel[2].percent < CRITICAL_LOW) {
        if (fuel[0].percent > CRITICAL_LOW) {
            if (fuel[1].percent > CRITICAL_LOW) {
                if (fuel[0].percent > fuel[1].percent) {
                    turn_on_pump_1();
                } else {
                    turn_on_pump_2();
                }
            } else {
                turn_on_pump_1();
            }
        } else if (fuel[1].percent > CRITICAL_LOW) {
            turn_on_pump_2();
        } else {
            turn_on_all_pumps();
        }
    } else {
        turn_on_pump_3();
    }
}

void fuel_auto_control()
{
    switch (pump_stage) {
    case 1:
        pump_stage_1();
        break;
    case 2:
        pump_stage_2();
        break;
    case 3:
        pump_stage_3();
        break;
    case 4:
        pump_stage_4();
        break;
    }
}

void check_answer_time()
{
    uint32_t now = time_ms();

    m_warn1::publish((fuel[0].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn2::publish((fuel[1].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn3::publish((fuel[2].time_ans + TIME_SA * 1000 < now) ? true : false);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel1::publish(fuel[0].liters);
    m_fuel2::publish(fuel[1].liters);
    m_fuel3::publish(fuel[2].liters);

    m_fuel_l::publish(fuel[0].liters + fuel[1].liters + fuel[2].liters);
    m_fuel_p::publish((fuel[0].percent + fuel[1].percent + fuel[2].percent) / 3.f);

    const bool ignition = (bool) m_pwr_eng::value();
    const bool fuel_mc = (bool) m_fuel_mc::value();
    const bool ers = (bool) m_ers1::value();

    if (ers) {
        turn_off_all_pumps();
    } else {
        if (ignition != ignition_old) {
            ignition_old = ignition;
            if (!ignition) {
                turn_off_all_pumps();
            }
        }

        if (fuel_mc != fuel_mc_old) {
            fuel_mc_old = fuel_mc;
            if (fuel_mc) {
                printf("Manual fuel ON");
            } else {
                turn_off_all_pumps();
                printf("Manual fuel OFF");
            }
        }

        if (!fuel_mc && ignition) {
            fuel_auto_control();
        }
    }

    //request fuel sensors
    snd_fuel_buf[0] = 0x31;
    snd_fuel_buf[2] = 0x06;

    snd_fuel_buf[1] = ADR_FUEL[idx_fuel_array++];
    if (idx_fuel_array == 3) {
        idx_fuel_array = 0;
    }

    snd_fuel_buf[3] = calcCRC(snd_fuel_buf, 3);
    send(port_fuel_id, snd_fuel_buf, 4, true);
}

EXPORT void on_fuel_serial(const uint8_t *data, size_t size)
{
    //typePack(0)[0x3E] adr(1) fmt(2) data(3) crc(8)
    if (size != MSG_FUEL_SIZE) {
        return;
    }

    if ((data[0] != 0x3E) || (data[2] != 0x06)) {
        return;
    }

    if (data[8] != calcCRC(data, 8)) {
        return;
    }

    float fuel_percent = (float) (data[4] | (data[5] << 8)) / 10.f;

    if (data[1] == ADR_FUEL[0]) {
        fuel[0].set_percent(fuel_percent);
    } else if (data[1] == ADR_FUEL[1]) {
        fuel[1].set_percent(fuel_percent);
    } else if (data[1] == ADR_FUEL[2]) {
        fuel[2].set_percent(fuel_percent);
    }
}
