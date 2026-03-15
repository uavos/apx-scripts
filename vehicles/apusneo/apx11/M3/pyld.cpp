#include <apx.h>

constexpr const char *txt_dev = "PYLD";

constexpr const uint8_t NODE_ID{7};
constexpr const uint8_t RECEIVER_ID{14};

constexpr const uint8_t MSG7_ID{6};

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
    int8_t temp_cam1;
    int8_t temp_cam2[2];
    int8_t temp_lens;
    int8_t temp_comp;
    int8_t temp_base_cam;
    int8_t temp_swmf1;
    t_srv srv_antenna;
    uint8_t crc;
};
#pragma pack()

PYLD_DATA _pyld = {};

constexpr const float ANT_MULT{-1.0f};
constexpr const float ANT_OFFSET{0.f};

bool auto_heat_state{true};

//get
using m_room = Mandala<mandala::sns::env::scr::s1>;
using m_cam1 = Mandala<mandala::sns::env::scr::s2>;
using m_cam2_pc = Mandala<mandala::sns::env::scr::s3>;
using m_lens = Mandala<mandala::sns::env::scr::s4>;
using m_cam2 = Mandala<mandala::sns::env::scr::s5>;
using m_comp = Mandala<mandala::sns::env::scr::s6>;
using m_base_cam = Mandala<mandala::sns::env::scr::s7>;
using m_swmf1 = Mandala<mandala::sns::env::scr::s8>;

using m_s10 = Mandala<mandala::sns::env::scr::s10>; //srv pos
using m_s11 = Mandala<mandala::sns::env::scr::s11>; //srv temp

//set
using m_cam1_h = Mandala<mandala::est::env::usrb::b10>; //cam1 heater
using m_cam2_h = Mandala<mandala::est::env::usrb::b11>; //cam2 heater
using m_lens_h = Mandala<mandala::est::env::usrb::b12>; //lens heater
using m_comp_h = Mandala<mandala::est::env::usrb::b13>; //comp heater

int main()
{
    m_room();
    m_cam1();
    m_cam2_pc();
    m_lens();
    m_cam2();
    m_comp();
    m_base_cam();
    m_swmf1();

    m_s10();
    m_s11();

    //header
    _pyld.header[0] = 0x4d;
    _pyld.header[1] = 0x41;
    _pyld.header[2] = RECEIVER_ID | ((NODE_ID << 4) & 0xF0);
    _pyld.header[3] = MSG7_ID;

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    task("pyld_heat"); // 0/1

    printf("%s Script ready...\n", txt_dev);

    return 0;
}

template<typename T>
T limit(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
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

void send_pyld_telemetry()
{
    //data
    _pyld.temp_room = (int8_t) (m_room::value());
    _pyld.temp_cam1 = (int8_t) (m_cam1::value());
    _pyld.temp_cam2[0] = (int8_t) (m_cam2::value());
    _pyld.temp_cam2[1] = (int8_t) m_cam2_pc::value();
    _pyld.temp_lens = (int8_t) m_lens::value();
    _pyld.temp_comp = (int8_t) m_comp::value();
    _pyld.temp_base_cam = (int8_t) m_base_cam::value();
    _pyld.temp_swmf1 = (int8_t) m_swmf1::value();

    int32_t ant_pos = int32_t(m_s10::value() * ANT_MULT + ANT_OFFSET);

    _pyld.srv_antenna.pos = (int8_t) limit(ant_pos, 0, 100);
    _pyld.srv_antenna.temp = (int8_t) m_s11::value();

    //crc
    _pyld.crc = calcTelemetryCRC(&_pyld.header[0], sizeof(PYLD_DATA) - 1);

    send(PORT_ID_GCU, &_pyld.header, sizeof(PYLD_DATA), true);
}

bool heater(int8_t temp, int8_t threshold_on, int8_t threshold_off)
{
    bool state{false};

    if (temp < threshold_on)
        state = true;
    else if (temp > threshold_off)
        state = false;

    return state;
}

EXPORT void on_heater()
{
    if (!auto_heat_state) {
        return;
    }

    bool state{false};

    state = heater((int8_t) m_cam1::value(), -20, -10);
    m_cam1_h::publish(state);

    state = heater((int8_t) ((m_cam2::value() + m_cam2_pc::value()) * 0.5f), -10, 0);
    m_cam2_h::publish(state);

    state = heater((int8_t) m_lens::value(), -15, -5);
    m_lens_h::publish(state);

    state = heater((int8_t) m_comp::value(), -20, -10);
    m_comp_h::publish(state);
}

EXPORT void on_main()
{
    send_pyld_telemetry();
    on_heater();
}

EXPORT void pyld_heat(int32_t val)
{
    auto_heat_state = (bool) val;
    printf("pyld_heat:%u\n", auto_heat_state);
}
