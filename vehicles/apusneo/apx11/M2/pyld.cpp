#include <apx.h>

constexpr const char *txt_dev = "PYLD";

constexpr const uint8_t NODE_ID{7};
constexpr const uint8_t RECEIVER_ID{14};

constexpr const uint8_t MSG6_ID{5};

constexpr const port_id_t PORT_ID_GCU{40};

constexpr const uint16_t TASK_MAIN_MS{1000};

#pragma pack(1)
typedef struct
{
    int8_t pos;
    int8_t temp;
} __attribute__((packed)) t_srv;
#pragma pack()

#pragma pack(1)
struct PYLD_DATA
{
    uint8_t header[4];
    int8_t temp_room;
    int8_t temp_cam[2];
    int8_t temp_lens;
    int8_t temp_comp[2];
    int8_t temp_swmf1;
    t_srv srv_antenna;
    t_srv srv_valv;
    uint8_t crc;
};
#pragma pack()

PYLD_DATA _pyld = {};

//get
using m_room = Mandala<mandala::sns::env::scr::s1>;
using m_cam1 = Mandala<mandala::sns::env::scr::s2>;
using m_cam2 = Mandala<mandala::sns::env::scr::s3>;
using m_lens = Mandala<mandala::sns::env::scr::s4>;
using m_comp1 = Mandala<mandala::sns::env::scr::s5>;
using m_comp2 = Mandala<mandala::sns::env::scr::s6>;
using m_swmf1 = Mandala<mandala::sns::env::scr::s7>;

using m_s10 = Mandala<mandala::sns::env::scr::s10>; //srv pos
using m_s11 = Mandala<mandala::sns::env::scr::s11>; //srv temp
using m_s12 = Mandala<mandala::sns::env::scr::s12>; //srv pos
using m_s13 = Mandala<mandala::sns::env::scr::s13>; //srv temp

int main()
{
    m_room();
    m_cam1();
    m_cam2();
    m_lens();
    m_comp1();
    m_comp2();
    m_swmf1();

    m_s10();
    m_s11();
    m_s12();
    m_s13();

    //header
    _pyld.header[0] = 0x4d;
    _pyld.header[1] = 0x41;
    _pyld.header[2] = RECEIVER_ID | ((NODE_ID << 4) & 0xF0);
    _pyld.header[3] = MSG6_ID;

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    printf("%s Script ready...\n", txt_dev);

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

EXPORT void on_main()
{
    //data
    _pyld.temp_room = (int8_t) (m_room::value());
    _pyld.temp_cam[0] = (int8_t) (m_cam1::value());
    _pyld.temp_cam[1] = (int8_t) m_cam2::value();
    _pyld.temp_lens = (int8_t) m_lens::value();
    _pyld.temp_comp[0] = (int8_t) m_comp1::value();
    _pyld.temp_comp[1] = (int8_t) m_comp2::value();
    _pyld.temp_swmf1 = (int8_t) m_swmf1::value();

    _pyld.srv_antenna.pos = (int8_t) (m_s10::value() * 1.f + 60.f);
    _pyld.srv_antenna.temp = (int8_t) m_s11::value();

    _pyld.srv_valv.pos = (int8_t) (m_s12::value() * 2.2f + 73.3f);
    _pyld.srv_valv.temp = (int8_t) m_s13::value();

    //printf("temp_comp1:%.2f", _pyld.temp_comp[0]);
    //printf("temp_comp2:%.2f", _pyld.temp_comp[1]);

    //crc
    _pyld.crc = calcTelemetryCRC(&_pyld.header[0], sizeof(PYLD_DATA) - 1);

    send(PORT_ID_GCU, &_pyld.header, sizeof(PYLD_DATA), true);
}
