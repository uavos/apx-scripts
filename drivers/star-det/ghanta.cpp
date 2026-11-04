#include <apx.h>

constexpr const char *txt_dev = "STAR";

constexpr const port_id_t PORT_ID_STAR{60}; //placeholder, real port id is not assigned yet

constexpr const uint16_t TASK_MAIN_MS{500}; //2Hz

constexpr const uint8_t MSG_MARK{0x5A};
constexpr const uint8_t MSG0_ID{0};
constexpr const uint16_t CRC16_INIT{0xFFFF}; //CCITT-FALSE, init value is not given in the protocol

//----------------------Message-----------------------------
// MARK | LEN | MSG_ID | DATA | CRC, little-endian
// LEN = MSG_ID + DATA, CRC16 covers LEN, MSG_ID and DATA

#pragma pack(1)
struct STAR_DATA
{
    uint8_t mark;
    uint16_t len;
    uint8_t msg_id;
    int16_t roll;   //0.01 deg
    int16_t pitch;  //0.01 deg
    uint16_t speed; //0.1 km/h
    int32_t lat;    //1e-7 deg
    int32_t lng;    //1e-7 deg
    uint32_t alt;   //0.1 m
    uint32_t ts;    //ms
    uint16_t crc;
};
#pragma pack()

STAR_DATA _star = {};

//----------------------Mandala------------------------------
using m_roll = Mandala<mandala::est::nav::att::roll>;   //rad
using m_pitch = Mandala<mandala::est::nav::att::pitch>; //rad
using m_speed = Mandala<mandala::est::nav::pos::speed>; //m/s, ground speed
using m_lat = Mandala<mandala::est::nav::pos::lat>;     //dword, 1e-7 deg
using m_lon = Mandala<mandala::est::nav::pos::lon>;     //dword, 1e-7 deg
using m_hmsl = Mandala<mandala::est::nav::pos::hmsl>;   //m, altitude MSL

int main()
{
    m_roll();
    m_pitch();
    m_speed();
    m_lat();
    m_lon();
    m_hmsl();

    _star.mark = MSG_MARK;
    _star.len = sizeof(STAR_DATA) - sizeof(_star.mark) - sizeof(_star.len) - sizeof(_star.crc);
    _star.msg_id = MSG0_ID;

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    printf("%s Script ready...\n", txt_dev);

    return 0;
}

uint16_t calcCRC16(const uint8_t *data, size_t size)
{
    uint16_t crc = CRC16_INIT;
    for (size_t i = 0; i < size; i++) {
        crc ^= (uint16_t) data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = crc & 0x8000 ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
    }
    return crc;
}

// scale, round and limit to the field range, NaN gives 0
float scale(float v, float k, float min, float max)
{
    v *= k;
    if (v != v)
        return 0.f;
    if (v < min)
        return min;
    if (v > max)
        return max;
    return floor(v + 0.5f);
}

// dword mandala value is read as float: int32 in 1e-7 deg is stored as uint32
int32_t gpsRaw(float v)
{
    if (v >= 2147483648.f)
        v -= 4294967296.f;
    return (int32_t) v;
}

EXPORT void on_main()
{
    _star.roll = (int16_t) scale(m_roll::value() * R2D, 100.f, -18000.f, 18000.f);
    _star.pitch = (int16_t) scale(m_pitch::value() * R2D, 100.f, -9000.f, 9000.f);
    _star.speed = (uint16_t) scale(m_speed::value() * 3.6f, 10.f, 0.f, 65535.f);
    _star.lat = gpsRaw(m_lat::value());
    _star.lng = gpsRaw(m_lon::value());
    _star.alt = (uint32_t) scale(m_hmsl::value(), 10.f, 0.f, 4e9f);
    _star.ts = time_ms();

    _star.crc = calcCRC16((const uint8_t *) &_star.len, sizeof(STAR_DATA) - sizeof(_star.mark) - sizeof(_star.crc));

    send(PORT_ID_STAR, &_star, sizeof(STAR_DATA), true);
}
