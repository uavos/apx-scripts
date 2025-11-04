#include <apx.h>

constexpr const char *txt_dev = "PYLD2";

constexpr const uint8_t NODE_ID{7};
constexpr const uint8_t RECEIVER_ID{14};

constexpr const uint8_t MSG7_ID{6};

constexpr const port_id_t PORT_ID_GCU{40};
constexpr const port_id_t PORT_ID_AGL{5};

constexpr const uint16_t TASK_MAIN_MS{100};

#pragma pack(1)
typedef struct
{
    int8_t pos;
    int8_t temp;
} __attribute__((packed)) t_srv;
#pragma pack()

#pragma pack(1)
struct PYLD_2_DATA
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

PYLD_2_DATA _pyld = {};

constexpr const float ANT_MULT{-1.0f};
constexpr const float ANT_OFFSET{0.f};

bool auto_heat_state{1};

//get
using m_room = Mandala<mandala::sns::env::scr::s1>;
using m_cam1 = Mandala<mandala::sns::env::scr::s2>;
using m_cam2_pc = Mandala<mandala::sns::env::scr::s3>;
using m_lens = Mandala<mandala::sns::env::scr::s4>;
using m_cam2 = Mandala<mandala::sns::env::scr::s5>;
using m_comp = Mandala<mandala::sns::env::scr::s6>;
using m_base_cam = Mandala<mandala::sns::env::scr::s7>;
//using m_swmf1 = Mandala<mandala::sns::env::scr::s8>;

using m_s10 = Mandala<mandala::sns::env::scr::s10>; //srv pos
using m_s11 = Mandala<mandala::sns::env::scr::s11>; //srv temp

//set
using m_cam1_h = Mandala<mandala::est::env::usrb::b10>; //cam1 heater
using m_cam2_h = Mandala<mandala::est::env::usrb::b11>; //cam2 heater
using m_lens_h = Mandala<mandala::est::env::usrb::b12>; //lens heater
using m_comp_h = Mandala<mandala::est::env::usrb::b13>; //comp heater

//AGL
using m_agl = Mandala<mandala::sns::nav::agl::laser>; //agl

constexpr const uint16_t SCHEDULE_PYLD2_TIMEOUT{1000};
uint32_t pyld2_tlm_timer{};

int main()
{
    m_room();
    m_cam1();
    m_cam2_pc();
    m_lens();
    m_cam2();
    m_comp();
    m_base_cam();
    //m_swmf1();

    m_s10();
    m_s11();

    //header
    _pyld.header[0] = 0x4d;
    _pyld.header[1] = 0x41;
    _pyld.header[2] = RECEIVER_ID | ((NODE_ID << 4) & 0xF0);
    _pyld.header[3] = MSG7_ID;

    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    receive(PORT_ID_AGL, "aglHandler");

    task("pld2_heat"); // 0/1

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

void send_pyld2_telemetry()
{
    //data
    _pyld.temp_room = (int8_t) (m_room::value());
    _pyld.temp_cam1 = (int8_t) (m_cam1::value());
    _pyld.temp_cam2[0] = (int8_t) (m_cam2::value());
    _pyld.temp_cam2[1] = (int8_t) m_cam2_pc::value();
    _pyld.temp_lens = (int8_t) m_lens::value();
    _pyld.temp_comp = (int8_t) m_comp::value();
    _pyld.temp_base_cam = (int8_t) m_base_cam::value();
    //_pyld.temp_swmf1 = (int8_t) m_swmf1::value();

    int32_t ant_pos = int32_t(m_s10::value() * ANT_MULT + ANT_OFFSET);

    _pyld.srv_antenna.pos = (int8_t) limit(ant_pos, 0, 100);
    _pyld.srv_antenna.temp = (int8_t) m_s11::value();

    printf("agl:%u", _pyld.srv_antenna.pos);

    //crc
    _pyld.crc = calcTelemetryCRC(&_pyld.header[0], sizeof(PYLD_2_DATA) - 1);

    send(PORT_ID_GCU, &_pyld.header, sizeof(PYLD_2_DATA), true);
}

bool heater(int8_t temp, int8_t threshold_on, int8_t threshold_off)
{
    if (temp < threshold_on)
        return true;

    if (temp > threshold_off)
        return false;

    return false;
}

EXPORT void on_heater()
{
    if (auto_heat_state == 0) {
        return;
    }

    bool state = heater((int8_t) m_cam1::value(), -20, -10);
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
    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, true);

    uint32_t now = time_ms();
    if (now - pyld2_tlm_timer > SCHEDULE_PYLD2_TIMEOUT) {
        pyld2_tlm_timer = now;
        send_pyld2_telemetry();
        on_heater();
    }
}

//ASCII:ld,0:-0.66
//HEX:6c 64 2c 30 3a 2d 30 2e 36 36 20 0d 0a
EXPORT void aglHandler(const uint8_t *data, size_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        if (data[i] == ':') {
            int sign = 1;
            float result = 0.0f;
            float divisor = 10.0f;
            bool decimal = false;
            i++;
            if (data[i] == '-') {
                sign = -1;
                i++;
            }
            for (; i < size; i++) {
                if (data[i] >= '0' && data[i] <= '9') {
                    if (decimal) {
                        result += (data[i] - '0') / divisor;
                        divisor *= 10.0f;
                    } else {
                        result = result * 10.0f + (data[i] - '0');
                    }
                } else if (data[i] == '.') {
                    decimal = true;
                } else {
                    break;
                }
            }
            float agl = (float) sign * result;

            //printf("agl:%.2f", agl);
            m_agl::publish(agl);
        }
    }
}

EXPORT void pld2_heat(int32_t val)
{
    auto_heat_state = (bool) val;
    printf("pld2_heat:%u\n", auto_heat_state);
}
