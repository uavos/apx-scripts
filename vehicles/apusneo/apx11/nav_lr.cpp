#include <apx.h>

#define NODE_LEFT TRUE //nav-l
//#define NODE_RIGHT TRUE //nav-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L"; //L
constexpr const uint8_t NODE_ID{1};
constexpr const uint8_t MSG7_ID{6};
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R"; //R
constexpr const uint8_t NODE_ID{2};
constexpr const uint8_t MSG7_ID{6};
#endif

constexpr const port_id_t PORT_ID_WING{43};

constexpr const uint16_t TASK_TELEMETRY_MS{200}; //5Hz
constexpr const uint16_t TASK_HEATER_MS{2000};   //0.5Hz

//----------------------Telemetry---------------------------

#pragma pack(1)
struct WING_DATA
{
    uint8_t header;
    uint16_t volatage[2]; //nav, srv
    int8_t volz_temp[2];
    uint16_t volz_pos[2];
    int8_t gyro_temp;
    uint8_t crc;
};
#pragma pack()

WING_DATA _wing = {};

//----------------------Temperature-------------------------
constexpr const uint16_t START_HEATER{10};

#if defined NODE_LEFT
using m_sw = Mandala<mandala::ctr::env::sw::sw1>; //heater off/on
#endif

#if defined NODE_RIGHT
using m_sw = Mandala<mandala::ctr::env::sw::sw5>; //heater off/on
#endif

//----------------------Mandala------------------------------
using m_f1 = Mandala<mandala::est::env::usrf::f1>; //volz temp
using m_f2 = Mandala<mandala::est::env::usrf::f2>; //volz temp
using m_f3 = Mandala<mandala::est::env::usrf::f3>; //volz pos
using m_f4 = Mandala<mandala::est::env::usrf::f4>; //volz pos

using m_f11 = Mandala<mandala::est::env::usrf::f11>; //pwr nav
using m_f12 = Mandala<mandala::est::env::usrf::f12>; //pwr srv

using m_sns_temp = Mandala<mandala::sns::nav::gyro::temp>; //gyro temp

int main()
{
    m_f1();
    m_f2();
    m_f3();
    m_f4();

    m_f11();
    m_f12();

    m_sns_temp();

    m_sw();

    task("on_telemetry", TASK_TELEMETRY_MS); //5 Hz
    task("on_heater", TASK_HEATER_MS);       //0.5 Hz

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
    //header
    _wing.header = MSG7_ID | ((NODE_ID << 4) & 0xF0);

    //data
    _wing.volatage[0] = (uint16_t) (m_f11::value() * 100.f);
    _wing.volatage[1] = (uint16_t) (m_f12::value() * 100.f);
    _wing.volz_temp[0] = (int8_t) m_f1::value();
    _wing.volz_temp[1] = (int8_t) m_f2::value();
    _wing.volz_pos[0] = (uint16_t) m_f3::value();
    _wing.volz_pos[1] = (uint16_t) m_f4::value();
    _wing.gyro_temp = (int8_t) m_sns_temp::value();

    //crc
    _wing.crc = calcTelemetryCRC(&_wing.header, sizeof(WING_DATA) - 1);

    send(PORT_ID_WING, &_wing.header, sizeof(WING_DATA), true);
}

EXPORT void on_heater()
{
    float RT = m_sns_temp::value();

    if (RT < START_HEATER) {
        m_sw::publish(1u);
    }

    if (RT > START_HEATER * 1.5f) {
        m_sw::publish(0u);
    }
}
