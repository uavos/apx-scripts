//Fuel control protocol
//UART: 19200 8 N 1

#include <apx.h>

//#define EQUAL_TANKS

static constexpr const port_id_t port_fuel_id{11};

const uint16_t TASK_MAIN_MS{100}; //msec
const uint16_t TASK_FUEL_MS{500}; //msec

//FUEL
const uint8_t ADR_FUEL_SENS1{85};
const uint8_t ADR_FUEL_SENS2{86};
const uint8_t ADR_FUEL_SENS3{87};

const uint8_t MSG_FUEL_SIZE{9}; //FUEL

#ifdef EQUAL_TANKS
const float V_MAX1{16.7f}; //liters
const float V_MAX2{16.7f}; //liters
const float V_MAX3{16.7f}; //liters

const float CRITICAL_LOW{7.0f};  //7% = 1.1l
const float TANK_1_POINT{13.0f}; //13% = 2l
const float TANK_2_POINT{25.0f}; //25% = 3.9l
const float TANK_3_POINT{51.0f}; //51% = 8l
#else
const float V_MAX1{17.0f}; //liters
const float V_MAX2{18.3f}; //liters
const float V_MAX3{24.6f}; //liters

const float CRITICAL_LOW{7.0f};  //7% = 1.1l
const float TANK_1_POINT{20.0f}; //20% = 3.4l
const float TANK_2_POINT{25.0f}; //25% = 4.5l
const float TANK_3_POINT{60.0f}; //60% = 14.7l
#endif

uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};

uint8_t ADR_FUEL[3] = {ADR_FUEL_SENS1, ADR_FUEL_SENS2, ADR_FUEL_SENS3};
uint8_t idx_fuel_array = 0;

uint32_t startTimerPumpON;

uint8_t pump_stage{1}; //default stage

const uint8_t TIME_SA{5};     //sec
const uint8_t DELAY_PUMP{15}; //sec between different pumps

bool fuel_mc_old{false};
bool ignition_old{false};
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

_fuel fuel[3] = {_fuel(V_MAX1), _fuel(V_MAX2), _fuel(V_MAX3)};

//ERS
uint32_t timer_ers = 0;

using m_ignition = Mandala<mandala::ctr::env::pwr::eng>;
using m_thr = Mandala<mandala::ctr::nav::eng::thr>;

//fuel
using m_fuel1 = Mandala<mandala::est::env::usr::u5>;
using m_fuel2 = Mandala<mandala::est::env::usr::u6>;
using m_fuel3 = Mandala<mandala::est::env::usr::u7>;

using m_fuel_p = Mandala<mandala::sns::env::fuel::level>;
using m_fuel_l = Mandala<mandala::est::env::usr::u8>;

using m_pump1 = Mandala<mandala::ctr::env::sw::sw5>;
using m_pump2 = Mandala<mandala::ctr::env::sw::sw6>;
using m_pump3 = Mandala<mandala::ctr::env::sw::sw7>;

using m_warn1 = Mandala<mandala::est::env::usrb::b5>;
using m_warn2 = Mandala<mandala::est::env::usrb::b6>;
using m_warn3 = Mandala<mandala::est::env::usrb::b7>;

using m_fuel_mc = Mandala<mandala::ctr::env::sw::sw8>; //manual control

//safety
using m_ltt = Mandala<mandala::est::env::sys::ltt>;
using m_health = Mandala<mandala::est::env::sys::health>;
using m_mode = Mandala<mandala::cmd::nav::proc::mode>;

//ers
using m_ers1 = Mandala<mandala::ctr::env::ers::launch>;
using m_ers2 = Mandala<mandala::est::env::usrb::b1>;

int main()
{
    schedule_periodic(task("on_main"), TASK_MAIN_MS);
    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    receive(port_fuel_id, "on_fuel_serial");

    m_ltt();
    m_health();

    m_ers1();
    m_ignition();
    m_thr();

    m_fuel1();
    m_fuel2();
    m_fuel3();
    m_fuel_p();
    m_fuel_l();
    m_pump1();
    m_pump2();
    m_pump3();
    m_warn1();
    m_warn2();
    m_warn3();
    m_fuel_mc();

    timer_ers = time_ms();

    return 0;
}

EXPORT void on_main()
{
    //Safety
    if ((uint32_t) m_ltt::value() < 10) {
        m_health::publish((uint32_t) mandala::sys_health_normal);
    }

    if ((uint32_t) m_health::value() == mandala::sys_health_warning) {
        m_mode::publish((uint32_t) mandala::proc_mode_LANDING);
    }

    //ERS
    if ((uint32_t) m_ers1::value() == mandala::ers_launch_on) {
        if ((time_ms() - timer_ers) > 3000) {
            m_ers2::publish(1u);
        }
    } else {
        timer_ers = time_ms();
        m_ers2::publish(0u);
    }
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

void turn_off_all_pumps()
{
    m_pump1::publish(0u);
    m_pump2::publish(0u);
    m_pump3::publish(0u);
}

void turn_on_all_pumps()
{
    m_pump1::publish(1u);
    m_pump2::publish(1u);
    m_pump3::publish(1u);
}

void turn_on_pump_1()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump2::value() || (bool) m_pump3::value()) {
            m_pump2::publish(0u);
            m_pump3::publish(0u);
            return;
        }
        if (!(bool) m_pump1::value()) {
            m_pump1::publish(1u);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_2()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value() || (bool) m_pump3::value()) {
            m_pump1::publish(0u);
            m_pump3::publish(0u);
            return;
        }
        if (!(bool) m_pump2::value()) {
            m_pump2::publish(1u);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_3()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value() || (bool) m_pump2::value()) {
            m_pump1::publish(0u);
            m_pump2::publish(0u);
            return;
        }
        if (!(bool) m_pump3::value()) {
            m_pump3::publish(1u);
            startTimerPumpON = time_ms();
        }
    }
}

void pump_stage_1() //get fuel from tank 3 until the point
{
    if (fuel[0].percent > TANK_3_POINT) {
        turn_on_pump_3();
    } else {
        pump_stage = 2;
        printf("fuel stage: 2");
    }
}

void pump_stage_2() //get fuel from tank 3 until empty and from 1 until the point
{
    if (fuel[0].percent > TANK_1_POINT) {
        if (fuel[2].percent > CRITICAL_LOW) {
            if (fuel[0].percent > (fuel[2].percent + 100 - TANK_3_POINT)) {
                turn_on_pump_1();
            } else {
                turn_on_pump_3();
            }
        } else {
            turn_on_pump_1();
        }
    } else {
        pump_stage = 3;
        printf("fuel stage: 3");
    }
}

void pump_stage_3() //get fuel from tank 2 until specified points
{
    if (fuel[2].percent < CRITICAL_LOW) {
        if (fuel[1].percent > TANK_2_POINT) {
            turn_on_pump_2();
        } else {
            pump_stage = 4;
            printf("fuel stage: 4");
        }
    } else {
        turn_on_pump_3();
    }
}

void pump_stage_4() //get fuel from tank 1 and 2 until empty
{
    if (fuel[2].percent < CRITICAL_LOW) {
        if (fuel[0].percent > CRITICAL_LOW) {
            if (fuel[1].percent > CRITICAL_LOW) {
                if (fuel[0].percent > fuel[1].percent) {
                    turn_on_pump_1();
                } else {
                    turn_on_pump_2();
                }
            } else {
                turn_on_pump_1();
            }
        } else if (fuel[1].percent > CRITICAL_LOW) {
            turn_on_pump_2();
        } else {
            turn_on_all_pumps();
        }
    } else {
        turn_on_pump_3();
    }
}

void fuel_auto_control()
{
    if (m_thr::value() > 0.85f) {
        turn_on_all_pumps();
        return;
    }
    switch (pump_stage) {
    case 1:
        pump_stage_1();
        break;
    case 2:
        pump_stage_2();
        break;
    case 3:
        pump_stage_3();
        break;
    case 4:
        pump_stage_4();
        break;
    }
}

void check_answer_time()
{
    uint32_t now = time_ms();

    m_warn1::publish((fuel[0].time_ans + TIME_SA * 1000 < now) ? 1u : 0u);
    m_warn2::publish((fuel[1].time_ans + TIME_SA * 1000 < now) ? 1u : 0u);
    m_warn3::publish((fuel[2].time_ans + TIME_SA * 1000 < now) ? 1u : 0u);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel1::publish(fuel[0].liters);
    m_fuel2::publish(fuel[1].liters);
    m_fuel3::publish(fuel[2].liters);

    m_fuel_l::publish(fuel[0].liters + fuel[1].liters + fuel[2].liters);
    m_fuel_p::publish((fuel[0].percent + fuel[1].percent + fuel[2].percent) / 3.f);

    const bool ignition = (bool) m_ignition::value();
    const bool fuel_mc = (bool) m_fuel_mc::value();
    const bool ers = (bool) m_ers1::value();

    if (ers) {
        turn_off_all_pumps();
    } else {
        if (ignition != ignition_old) {
            ignition_old = ignition;
            if (!ignition) {
                turn_off_all_pumps();
            }
        }

        if (fuel_mc != fuel_mc_old) {
            fuel_mc_old = fuel_mc;
            if (fuel_mc) {
                printf("Manual fuel ON");
            } else {
                turn_off_all_pumps();
                printf("Manual fuel OFF");
            }
        }

        if (!fuel_mc && ignition) {
            fuel_auto_control();
        }
    }

    //request fuel sensors
    snd_fuel_buf[0] = 0x31;
    snd_fuel_buf[2] = 0x06;

    snd_fuel_buf[1] = ADR_FUEL[idx_fuel_array++];
    if (idx_fuel_array == 3) {
        idx_fuel_array = 0;
    }

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

    if (data[1] == ADR_FUEL[0]) {
        fuel[0].set_percent(fuel_percent);
    } else if (data[1] == ADR_FUEL[1]) {
        fuel[1].set_percent(fuel_percent);
    } else if (data[1] == ADR_FUEL[2]) {
        fuel[2].set_percent(fuel_percent);
    }
}
