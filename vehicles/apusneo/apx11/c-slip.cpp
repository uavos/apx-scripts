#include <apx.h>

constexpr const uint8_t TASK_AGL_MS{100};

constexpr const port_id_t PORT_ID_AGL{5};

using m_agl = Mandala<mandala::sns::nav::agl::laser>; //agl

int main()
{
    task("on_task_agl", TASK_AGL_MS);

    receive(PORT_ID_AGL, "aglHandler");

    return 0;
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

EXPORT void on_task_agl()
{
    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, true);
}
