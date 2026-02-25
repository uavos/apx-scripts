// Script for fuel management in Arrow APX11
// For system where engine takes fuel from tank 2, which is refilled from tanks 1 and 3

#include <apx.h>

#define EQUAL_TANKS

#define FUEL_SIM
#define SIM_SPEED 0.03f

static constexpr const port_id_t port_fuel_id{11};
const uint16_t TASK_FUEL_MS{500};    //msec
const uint16_t TASK_FUEL_SIM_MS{50}; //msec

//FUEL
const uint8_t ADR_FUEL_SENS1{85}; //75
const uint8_t ADR_FUEL_SENS2{86}; //76
const uint8_t ADR_FUEL_SENS3{87}; //77

const uint8_t MSG_FUEL_SIZE{9}; //FUEL

#ifdef EQUAL_TANKS
const float V_MAX1{100.0f}; //liters
const float V_MAX2{100.0f}; //liters
const float V_MAX3{100.0f}; //liters
#endif

const float CRITICAL_LOW{7.0f};            //%, reaching this level considered empty tank
const float TANK_2_KEEPFULL{95.f};         //%, if below, start pumping fuel in tank 2
const float TANK_2_POINT{35.f};            //%, starting next stage when reaching this point
const float TANK_12_RATIO{2.f};            //ratio of fuel to keep between tank 1 and 2
const float TANK_13_MAX_DIFF{20.f};        //%, max allowed difference between tank 1 and 3 levels
const float TANK_13_MIN_DIFF{0.0f};        //%, min required difference between tank 1 and 3 levels
const float TANK_13_MAX_DIFF_LEVELS{50.f}; //%, max difference between tank 1 and 3 at 50% level
const float TANK_13_MIN_DIFF_LEVELS{15.f}; //%, min difference between tank 1 and 3 at 15% level

const float k1 = (TANK_13_MAX_DIFF - TANK_13_MIN_DIFF) / (TANK_13_MAX_DIFF_LEVELS - 100.f);
const float b1 = TANK_13_MIN_DIFF - k1 * 100.f;

const float k2 = (TANK_13_MIN_DIFF - TANK_13_MAX_DIFF)
                 / (TANK_13_MIN_DIFF_LEVELS - TANK_13_MAX_DIFF_LEVELS);
const float b2 = TANK_13_MAX_DIFF - k2 * TANK_13_MAX_DIFF_LEVELS;

uint8_t snd_fuel_buf[MSG_FUEL_SIZE] = {};

uint8_t ADR_FUEL[3] = {ADR_FUEL_SENS1, ADR_FUEL_SENS2, ADR_FUEL_SENS3};
uint8_t idx_fuel_array = 0;

uint32_t startTimerPumpON; //time when pump was turned on, used for switching between pumps

uint8_t pump_stage{1}; //default stage

const uint8_t TIME_SA{5}; //sec. waiting time for answer from fuel sensors, after which warning is on

#ifdef FUEL_SIM
const uint8_t DELAY_PUMP{4};     //pump works for 4 sec before switching to another
const uint8_t TIME_TO_EMPTY{20}; //sec. Time to drain tank from critical to empty level
#else
const uint8_t DELAY_PUMP{15};    //pump works for 15 sec before switching to another
const uint8_t TIME_TO_EMPTY{60}; //sec. Time to drain tank from critical to empty level
#endif

bool tank1_critical{false};
bool tank3_critical{false};
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

//Engine
using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_eng_cut = Mandala<mandala::cmd::nav::eng::cut>;

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

//ers
using m_ers1 = Mandala<mandala::ctr::env::ers::launch>;

int main()
{
#ifdef FUEL_SIM
    fuel[0].set_percent(100.f);
    fuel[1].set_percent(100.f);
    fuel[2].set_percent(100.f);
    schedule_periodic(task("on_fuel_sim"), TASK_FUEL_SIM_MS);
#endif
    printf("fuel stage: 1");
    schedule_periodic(task("on_fuel"), TASK_FUEL_MS);

    //receive(port_fuel_id, "on_fuel_serial");

    m_fuel_p();
    m_fuel_l();

    m_fuel1();
    m_fuel2();
    m_fuel3();

    m_pump1();
    m_pump2();
    m_pump3();

    m_warn1();
    m_warn2();
    m_warn3();

    m_fuel_mc();

    //ers
    m_ers1();
    m_pwr_eng();

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

//todo: restart fuel algorithm after launch

void turn_off_all_pumps()
{
    m_pump1::publish(false);
    m_pump2::publish(false);
    m_pump3::publish(false);
}

void turn_on_all_pumps()
{
    m_pump1::publish(true);
    m_pump2::publish(true);
    m_pump3::publish(true);
}

void turn_off_pump_1()
{
    m_pump1::publish(false);
}

void turn_off_pump_3()
{
    m_pump3::publish(false);
}

void turn_on_pump_1()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump3::value()) { //if pump 3 is on, turn it off before turning on pump 1
            turn_off_pump_3();
            return;
        }
        if (!(bool) m_pump1::value()) { //if pump 1 is not on, turn it on and start timer
            m_pump1::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void turn_on_pump_2()
{
    m_pump2::publish(true);
}

void turn_on_pump_3()
{
    if (time_ms() > startTimerPumpON + DELAY_PUMP * 1000) {
        if ((bool) m_pump1::value()) { //if pump 1 is on, turn it off before turning on pump 3
            turn_off_pump_1();
            return;
        }
        if (!(bool) m_pump3::value()) { //if pump 3 is not on, turn it on and start timer
            m_pump3::publish(true);
            startTimerPumpON = time_ms();
        }
    }
}

void pump_stage_1() //keep tank 2 full, pumping from 1 and 3 according to required difference
{                   // prioritize keeping more fuel in tank 1
    if (fuel[1].percent < TANK_2_KEEPFULL) {
        float avrg_t1t3 = (fuel[0].percent + fuel[2].percent) / 2.f;

        float req_diff = 0.0f;
        bool valid = true;

        if (avrg_t1t3 > TANK_13_MAX_DIFF_LEVELS) {
            req_diff = k1 * avrg_t1t3 + b1;
        } else if (avrg_t1t3 > TANK_13_MIN_DIFF_LEVELS) {
            req_diff = k2 * avrg_t1t3 + b2;
        } else {
            valid = false;
            pump_stage = 2; //if both 1 and 3 tanks are near 15%
            printf("fuel stage: 2");
            turn_off_pump_1(); //turn off pump 1
        }

        if (valid) {
            //keep required difference between tank 1 and 3
            if (fuel[0].percent - fuel[2].percent > req_diff) {
                turn_on_pump_1();
            } else {
                turn_on_pump_3();
            }
        }
    } else {
        turn_off_pump_1(); //does this prevent pumps from working for 15 sec???
        turn_off_pump_3();
    }
}

void pump_stage_2() //leaving tank 1 with around 15%, pump from tank 3 until empty
{
    if (!tank3_critical) {
        if (fuel[1].percent < TANK_2_KEEPFULL) { //if tank 2 is below 95%, pump fuel
            turn_on_pump_3();
        } else { //if tank 2 is full, turn off pumps and wait until fuel drops below 95% again
            turn_off_pump_1();
            turn_off_pump_3();
        }

        //if tank 3 near empty - start pump 3 for 1 minute to drain it for sure
        if (fuel[2].percent < CRITICAL_LOW) {
            tank3_critical = true;
            //force restart pump3 and it's timer immediately
            m_pump1::publish(false);
            m_pump3::publish(true);
            startTimerPumpON = time_ms();
        }
    } else if (time_ms() > startTimerPumpON + TIME_TO_EMPTY * 1000) {
        pump_stage = 3;
        printf("fuel stage: 3");
        turn_off_pump_3();
    }
}

void pump_stage_3() //leaving tank 1 with around 15%, pump from tank 2 until ~35%
{                   //pump 2 works when engine works, so no need to turn it on specifically
    if (fuel[1].percent < TANK_2_POINT) {
        pump_stage = 4;
        printf("fuel stage: 4");
    }
}

void pump_stage_4() // pump from tank 1 and tank 2 with specific ratio
{
    if (!tank1_critical) {
        if (fuel[1].percent > fuel[0].percent * TANK_12_RATIO) {
            turn_off_pump_1();
        } else {
            turn_on_pump_1();
        }

        if (fuel[0].percent < CRITICAL_LOW) {
            tank1_critical = true;
            //force restart pump1 and it's timer immediately
            m_pump3::publish(false);
            m_pump1::publish(true);
            startTimerPumpON = time_ms();
        }
    } else if (time_ms() > startTimerPumpON + TIME_TO_EMPTY * 1000) {
        turn_off_pump_1();
    }
}

void fuel_auto_control()
{
    turn_on_pump_2(); //keep pump2 on while engine is on

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

    m_warn1::publish((fuel[0].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn2::publish((fuel[1].time_ans + TIME_SA * 1000 < now) ? true : false);
    m_warn3::publish((fuel[2].time_ans + TIME_SA * 1000 < now) ? true : false);
}

EXPORT void on_fuel()
{
    check_answer_time();

    m_fuel1::publish(fuel[0].liters);
    m_fuel2::publish(fuel[1].liters);
    m_fuel3::publish(fuel[2].liters);

    m_fuel_l::publish(fuel[0].liters + fuel[1].liters + fuel[2].liters);
    m_fuel_p::publish((fuel[0].percent + fuel[1].percent + fuel[2].percent) / 3.f);

    const bool ignition = (bool) m_pwr_eng::value();
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

EXPORT void on_fuel_sim()
{
    if ((bool) m_pump2::value()) { //simulate fuel flow from tank 2 to engine
        if (fuel[1].percent > 3.f) {
            fuel[1].set_percent(fuel[1].percent - SIM_SPEED * 0.95f);
        } else {
            fuel[1].set_percent(3.f);
        }
    }

    if ((bool) m_pump1::value()) {
        if (fuel[0].percent > 3.f) { //simulate fuel flow from tank 1 to tank 2
            fuel[0].set_percent(fuel[0].percent - SIM_SPEED);
            fuel[1].set_percent(fuel[1].percent + SIM_SPEED);
        } else {
            fuel[0].set_percent(3.f);
        }
    }

    if ((bool) m_pump3::value()) {
        if (fuel[2].percent > 3.f) { //simulate fuel flow from tank 3 to tank 2
            fuel[2].set_percent(fuel[2].percent - SIM_SPEED);
            fuel[1].set_percent(fuel[1].percent + SIM_SPEED);
        } else {
            fuel[2].set_percent(3.f);
        }
    }
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
