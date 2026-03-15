#include <apx.h>

constexpr const uint8_t TASK_AGL_MS{100};

constexpr const port_id_t PORT_ID_AGL{5};
constexpr const port_id_t PORT_ID_AIR{6};

//AIR
uint8_t MSG_AIR_SIZE = 35;
uint8_t AIR_DATA[35] = {};

float air_spd = 0.f;
float air_hdg = 0.f;
float air_state = 0.f;
float air_temp = 0.f;

using m_spd = Mandala<mandala::est::env::usr::u5>; //spd
using m_hdg = Mandala<mandala::est::env::usr::u6>; //hdg
using m_tmp = Mandala<mandala::est::env::usr::u7>; //temp

//AGL
using m_agl = Mandala<mandala::sns::nav::agl::laser>; //agl

int main()
{
    schedule_periodic(task("on_agl"), TASK_AGL_MS);

    receive(PORT_ID_AGL, "aglHandler");
    receive(PORT_ID_AIR, "airHandler");

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

float wrap(float x, float low, float high)
{
    float range = high - low;
    if (range <= 0.0f)
        return low;

    while (x < low)
        x += range;

    while (x >= high)
        x -= range;

    return x;
}

EXPORT void on_agl()
{
    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, true);
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

//$WI,WVC=000.0,137,0,+027.4,C,A*4C
EXPORT void airHandler(const uint8_t *data, size_t size)
{
    if (size != MSG_AIR_SIZE) {
        return;
    }

    memcpy(&AIR_DATA, data, MSG_AIR_SIZE);

    if (AIR_DATA[0] != '$' || AIR_DATA[1] != 'W' || AIR_DATA[2] != 'I') {
        return;
    }

    uint8_t indx = 0;
    uint8_t v[4] = {};

    //---
    while (AIR_DATA[indx] != 0x2C) {
        indx++;
    }

    v[0] = AIR_DATA[indx + 5] - '0';
    v[1] = AIR_DATA[indx + 6] - '0';
    v[2] = AIR_DATA[indx + 7] - '0';
    v[3] = AIR_DATA[indx + 9] - '0';

    air_spd = v[0] * 100 + v[1] * 10 + v[2] * 1 + v[3] * 0.1f;

    //---
    indx++;
    while (AIR_DATA[indx] != 0x2C) {
        indx++;
    }

    v[0] = AIR_DATA[indx + 1] - '0';
    v[1] = AIR_DATA[indx + 2] - '0';
    v[2] = AIR_DATA[indx + 3] - '0';

    air_hdg = v[0] * 100 + v[1] * 10 + v[2];

    //---
    indx++;
    while (AIR_DATA[indx] != 0x2C) {
        indx++;
    }

    air_state = AIR_DATA[indx + 1] - '0';

    //---
    indx++;
    while (AIR_DATA[indx] != 0x2C) {
        indx++;
    }
    v[0] = AIR_DATA[indx + 2] - '0';
    v[1] = AIR_DATA[indx + 3] - '0';
    v[2] = AIR_DATA[indx + 4] - '0';
    v[3] = AIR_DATA[indx + 6] - '0';

    air_temp = v[0] * 100 + v[1] * 10 + v[2] * 1 + v[3] * 0.1f;

    if (AIR_DATA[indx + 1] == '-') {
        air_temp = -air_temp;
    }

    //printf("air:%.2f", air_spd);
    //printf("hdg:%.2f", air_hdg);
    //printf("state:%.1f", air_state);
    //printf("temp:%.2f", air_temp);

    m_spd::publish(air_spd);
    m_tmp::publish(air_temp);

    air_hdg = wrap(limit(air_hdg, 0.f, 360.f), -180.f, 180.f);
    m_hdg::publish(air_hdg);
}
