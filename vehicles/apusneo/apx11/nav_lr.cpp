#include <apx.h>

#define NODE_LEFT TRUE //L-TRUE R-comment
//#define NODE_RIGHT              TRUE            //R-TRUE L-comment

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

constexpr const uint16_t TASK_DELAY_MS{100}; //10Hz

//----------------------Telemetry---------------------------
constexpr const uint8_t MSG7_SIZE = 1 + 5 + 1; //header + data + crc
uint8_t telemetryPack[MSG7_SIZE] = {};
//----------------------------------------------------------

using m_f1 = Mandala<mandala::est::env::usrf::f1>; //volz pos
using m_f2 = Mandala<mandala::est::env::usrf::f2>; //volz pos
using m_f3 = Mandala<mandala::est::env::usrf::f3>; //volz temp
using m_f4 = Mandala<mandala::est::env::usrf::f4>; //volz temp

using m_sns_temp = Mandala<mandala::sns::nav::gyro::temp>; //gyro temp

#pragma pack(1)
struct WING_DATA
{
    int8_t volz_pos[2];
    int8_t volz_temp[2];
    int8_t gyro_temp;
};
#pragma pack()

WING_DATA _wing = {};

int main()
{
    m_f1();
    m_f2();
    m_f3();
    m_f4();

    m_sns_temp();

    task("on_task", TASK_DELAY_MS); //10 Hz

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

void sendWingTelemetry()
{
    //header
    telemetryPack[0] = MSG7_ID | ((NODE_ID << 4) & 0xF0);

    //data
    memcpy(&telemetryPack[1], &_wing, sizeof(WING_DATA));

    //crc
    uint8_t crc = calcTelemetryCRC(telemetryPack, MSG7_SIZE - 1);
    telemetryPack[MSG7_SIZE - 1] = crc;

    send(PORT_ID_WING, telemetryPack, MSG7_SIZE, true);
}

EXPORT void on_task()
{
    _wing.volz_pos[0] = (int8_t) m_f1::value();
    _wing.volz_pos[1] = (int8_t) m_f2::value();
    _wing.volz_temp[0] = (int8_t) m_f3::value();
    _wing.volz_temp[1] = (int8_t) m_f4::value();
    _wing.gyro_temp = (int8_t) m_sns_temp::value();

    sendWingTelemetry();
}
