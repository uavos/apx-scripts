#include <apx.h>

constexpr const uint16_t TASK_MAIN_MS{200}; //msec

constexpr const port_id_t PORT_ID_ESC{50};

struct ESC_VCP_Data
{
    uint8_t temp;
    float voltage;
    float current;
    uint16_t consumption;
    uint16_t rpm;
};

ESC_VCP_Data esc_data;

const uint8_t PACK_SIZE_ESC = 10;
uint8_t esc_tbuf[PACK_SIZE_ESC] = {};

using m_eng_temp = Mandala<mandala::sns::env::eng::temp>;
using m_eng_volt = Mandala<mandala::sns::env::eng::voltage>;
using m_eng_carent = Mandala<mandala::sns::env::eng::current>; // sns.eng.current
using m_eng_rpm = Mandala<mandala::sns::env::eng::rpm>;
using m_launch = Mandala<mandala::ctr::env::ers::launch>; //launch
using m_cmd_cut = Mandala<mandala::cmd::nav::eng::cut>;   //eng cut

//datalink
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_MS);

    receive(PORT_ID_ESC, "esc_handler");

    //datalink
    m_ltt();
    m_health();
    m_mode();

    m_launch("on_launch"); // subscribe

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

    //save data to mandala
    m_eng_temp::publish((uint32_t) esc_data.temp);
    m_eng_volt::publish((float) esc_data.voltage);
    m_eng_carent::publish((float) esc_data.current);
    m_eng_rpm::publish((uint32_t) esc_data.rpm);

    //printf("temp:%u", esc_data.temp);
    //printf("voltage:%.2f", esc_data.voltage);
    //printf("current:%.2f", esc_data.current);
    //printf("consumption:%u", esc_data.consumption);
    //printf("rpm:%u", esc_data.rpm);
}

uint8_t update_crc8(uint8_t data, uint8_t crc)
{
    data ^= crc;

    for (uint8_t i = 0; i < 8; i++) {
        data = uint8_t((data & 0x80) ? 0x07 ^ (data << 1) : (data << 1));
    }
    return data;
}

uint8_t get_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc{0};
    for (uint8_t i = 0; i < len; i++) {
        crc = update_crc8(data[i], crc);
    }
    return crc & 0xFF;
}

EXPORT void esc_handler(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_ESC) {
        return;
    }

    memcpy(esc_tbuf, data, size);

    if (get_crc8(esc_tbuf, PACK_SIZE_ESC - 1) != esc_tbuf[PACK_SIZE_ESC - 1]) {
        return;
    }

    esc_data.temp = data[0];
    esc_data.voltage = float((esc_tbuf[1] << 8) | (esc_tbuf[2])) / 100.f;
    esc_data.current = float((esc_tbuf[3] << 8) | (esc_tbuf[4])) / 100.f;
    esc_data.consumption = uint16_t((esc_tbuf[5] << 8) | (esc_tbuf[6]));
    esc_data.rpm = uint16_t((esc_tbuf[7] << 8) | (esc_tbuf[8])) * 100 / 7u;
}

EXPORT void on_launch()
{
    if (m_launch::value() == 1u) {
        sleep(100);
        m_cmd_cut::publish(true);
    }
}
