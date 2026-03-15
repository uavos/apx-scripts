#include <apx.h>

#define NODE_LEFT TRUE //nav-l
//#define NODE_RIGHT TRUE //nav-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L"; //L
constexpr const uint8_t NODE_ID{1};
constexpr const uint8_t RECEIVER_ID{3};
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R"; //R
constexpr const uint8_t NODE_ID{2};
constexpr const uint8_t RECEIVER_ID{4};
#endif

constexpr const uint8_t MSG4_ID{3};

constexpr const port_id_t PORT_ID_WING{43};

constexpr const uint16_t TASK_TELEMETRY_MS{200}; //5Hz
constexpr const uint16_t TASK_HEATER_MS{2000};   //0.5Hz

//----------------------Telemetry---------------------------

#pragma pack(1)
struct WING_DATA
{
    uint8_t header[4];
    uint16_t voltage[2]; //sys, srv
    int8_t volz_temp[2];
    int16_t volz_pos[2];
    int8_t gyro_temp;
    uint8_t crc;
};
#pragma pack()

WING_DATA _wing = {};

//----------------------Temperature-------------------------
constexpr const uint16_t START_HEATER{10};

using m_sw_manual = Mandala<mandala::ctr::env::sw::sw15>; //heater manual

#if defined NODE_LEFT
using m_sw = Mandala<mandala::ctr::env::sw::sw1>; //heater off/on
#endif

#if defined NODE_RIGHT
using m_sw = Mandala<mandala::ctr::env::sw::sw5>; //heater off/on
#endif

//----------------------Mandala------------------------------
using m_s1 = Mandala<mandala::sns::env::scr::s1>; //volz temp
using m_s2 = Mandala<mandala::sns::env::scr::s2>; //volz temp
using m_s3 = Mandala<mandala::sns::env::scr::s3>; //volz pos
using m_s4 = Mandala<mandala::sns::env::scr::s4>; //volz pos

using m_pwr_sys = Mandala<mandala::sns::env::pwr::vsys>; //pwr nav
using m_pwr_srv = Mandala<mandala::sns::env::pwr::vsrv>; //pwr srv

using m_sns_temp = Mandala<mandala::sns::nav::gyro::temp>; //gyro temp

int main()
{
    m_s1();
    m_s2();
    m_s3();
    m_s4();

    m_pwr_sys();
    m_pwr_srv();

    m_sns_temp();

    m_sw_manual();

    //header
    _wing.header[0] = 0x4d;
    _wing.header[1] = 0x41;
    _wing.header[2] = RECEIVER_ID | ((NODE_ID << 4) & 0xF0);
    _wing.header[3] = MSG4_ID;

    schedule_periodic(task("on_telemetry"), TASK_TELEMETRY_MS);
    schedule_periodic(task("on_heater"), TASK_HEATER_MS);

    printf("NAV-WING:%s Script ready...\n", txt_dev);

    return 0;
}

uint8_t calcTelemetryCRC(const uint8_t *data, uint8_t size)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = crc & 0x80 ? (uint8_t) (crc << 1) ^ 0x31 : (uint8_t) (crc << 1);
    }
    return crc;
}

EXPORT void on_telemetry()
{
    //data
    _wing.voltage[0] = (uint16_t) (m_pwr_sys::value() * 100.f);
    _wing.voltage[1] = (uint16_t) (m_pwr_srv::value() * 100.f);
    _wing.volz_temp[0] = (int8_t) m_s1::value();
    _wing.volz_temp[1] = (int8_t) m_s2::value();
    _wing.volz_pos[0] = (int16_t) m_s3::value();
    _wing.volz_pos[1] = (int16_t) m_s4::value();
    _wing.gyro_temp = (int8_t) m_sns_temp::value();

    //crc
    _wing.crc = calcTelemetryCRC(&_wing.header[0], sizeof(WING_DATA) - 1);

    send(PORT_ID_WING, &_wing.header, sizeof(WING_DATA), true);
}

EXPORT void on_heater()
{
    if ((bool) m_sw_manual::value()) {
        return;
    }

    float RT = m_sns_temp::value();

    if (RT < START_HEATER) {
        m_sw::publish(1u);
    }

    if (RT > START_HEATER * 1.5f) {
        m_sw::publish(0u);
    }
}
