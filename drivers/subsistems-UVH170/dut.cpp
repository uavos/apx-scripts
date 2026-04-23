//id 170
//UART: 19200 8 N 1

#include <apx.h>

static constexpr const port_id_t port_fuel_id{3};
const uint16_t TASK_FUEL_MS{500}; //msec
const uint8_t ADR_FUEL_SENS1{170};
const uint8_t MSG_FUEL_SIZE{9}; //FUEL
uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};
uint8_t ADR_FUEL[1] = {ADR_FUEL_SENS1};
const float V_MAX1 = 16.7f;
const uint8_t TIME_SA{5};     //sec

struct _fuel
{
    _fuel(float v_max)
        : V_MAX(v_max)
        , liters(0.f)
        , percent(0.f)
        , time_ans(0)
    {}

    void set_percent(float p)
    {
        percent = p;
        liters = (p * V_MAX) / 100.f;
        time_ans = time_ms();
    }

    float V_MAX;   // max tank liters
    float liters;  // liters
    float percent; // percent
    uint32_t time_ans;
};

_fuel fuel[1] = {_fuel(V_MAX1)};


//fuel
using m_fuel1 = Mandala<mandala::est::env::usr::u5>;

using m_fuel_p = Mandala<mandala::sns::env::fuel::level>;
using m_fuel_l = Mandala<mandala::est::env::usr::u8>;

using m_warn1 = Mandala<mandala::est::env::usrb::b5>;


int main()
{

    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    receive(port_fuel_id, "on_fuel_serial");

    //fuel
    m_fuel_p();
    m_fuel_l();
    m_fuel1();
    m_warn1();

    return 0;
}

EXPORT uint8_t calcCRC(const uint8_t *buf, size_t size)
{
    uint8_t crc = 0x00;

    for (uint8_t j = 0; j < size; j++) {
        uint8_t i = buf[j] ^ crc;
        crc = 0;
        if (i & 0x01)
            crc ^= 0x5e;
        if (i & 0x02)
            crc ^= 0xbc;
        if (i & 0x04)
            crc ^= 0x61;
        if (i & 0x08)
            crc ^= 0xc2;
        if (i & 0x10)
            crc ^= 0x9d;
        if (i & 0x20)
            crc ^= 0x23;
        if (i & 0x40)
            crc ^= 0x46;
        if (i & 0x80)
            crc ^= 0x8c;
    }
    return crc % 256;
}



void check_answer_time()
{
    uint32_t now = time_ms();

    m_warn1::publish((fuel[0].time_ans + TIME_SA * 1000 < now) ? true : false);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel1::publish(fuel[0].liters);

    m_fuel_l::publish(fuel[0].liters);
    m_fuel_p::publish(fuel[0].percent);


    //request fuel sensors
    snd_fuel_buf[0] = 0x31;
    snd_fuel_buf[2] = 0x06;

    snd_fuel_buf[1] = ADR_FUEL[0];

    snd_fuel_buf[3] = calcCRC(snd_fuel_buf, 3);
    send(port_fuel_id, snd_fuel_buf, 4, true);
}

EXPORT void on_fuel_serial(const uint8_t *data, size_t size)
{
    //typePack(0)[0x3E] adr(1) fmt(2) data(3) crc(8)
    if (size != MSG_FUEL_SIZE) {
        return;
    }

    if ((data[0] != 0x3E) || (data[2] != 0x06)) {
        return;
    }

    if (data[8] != calcCRC(data, 8)) {
        return;
    }

    float fuel_percent = (float) (data[4] | (data[5] << 8)) / 10.f;

    //printf("fuel: %.2f", fuel_percent);

    if (data[1] == ADR_FUEL[0]) {
        fuel[0].set_percent(fuel_percent);
    }
}
