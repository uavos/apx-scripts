//v1.2 multi-tank fuel balance script
#include <apx.h>

//#define DEBUG

//#define EQUAL_TANKS

static constexpr port_id_t port_id{240};
const uint8_t ADR_SENS1{75};
const uint8_t ADR_SENS2{76};
const uint8_t ADR_SENS3{77};
const uint8_t PACK_SIZE{9};

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

const uint8_t TIME_SA{2};             //sec
const uint8_t DELAY_PUMP{15};         //sec between different pumps
const uint16_t FUEL_TASK_PERIOD{200}; //msec

uint8_t msg[PACK_SIZE];
uint8_t m_sensRequest{0}; //0 - sens1, 1 - sens2, 2 - sens3

float m_vFuelPercent1;
float m_vFuelPercent2;
float m_vFuelPercent3;

uint32_t m_timeAns1;
uint32_t m_timeAns2;
uint32_t m_timeAns3;
uint32_t startTimerPumpON;
uint32_t ignitionTotalTime{0};

uint8_t pump_stage{1}; //default stage

bool ers;
bool algorithm; //0 - enable; 1 - disable
bool algorithmOld;
bool ignition;
bool ignitionOld;

using m_ignition = Mandala<mandala::ctr::env::pwr::eng>;
using m_thr = Mandala<mandala::ctr::nav::eng::thr>;
using m_ers = Mandala<mandala::ctr::env::ers::launch>;
using m_algorithm = Mandala<mandala::est::env::usrb::b1>;

using m_pump1 = Mandala<mandala::est::env::usrb::b2>;
using m_pump2 = Mandala<mandala::est::env::usrb::b3>;
using m_pump3 = Mandala<mandala::est::env::usrb::b4>;

using M_FUEL_P = Mandala<mandala::sns::env::fuel::level>;
using M_FUEL_V1 = Mandala<mandala::est::env::usr::u1>;
using M_FUEL_V2 = Mandala<mandala::est::env::usr::u2>;
using M_FUEL_V3 = Mandala<mandala::est::env::usr::u3>;
using M_FUEL_L = Mandala<mandala::est::env::usr::u4>;
using m_flowrate = Mandala<mandala::sns::env::fuel::rate>;

using M_WARN_ANSWER1 = Mandala<mandala::est::env::usrb::b5>;
using M_WARN_ANSWER2 = Mandala<mandala::est::env::usrb::b6>;
using M_WARN_ANSWER3 = Mandala<mandala::est::env::usrb::b7>;

#ifdef DEBUG
using m_thr = Mandala<mandala::cmd::nav::rc::thr>;
#endif

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
    if (m_vFuelPercent3 > TANK_3_POINT) {
        turn_on_pump_3();
    } else {
        pump_stage = 2;
        printf("fuel stage: 2");
    }
}

void pump_stage_2() //get fuel from tank 3 until empty and from 1 until the point
{
    if (m_vFuelPercent1 > TANK_1_POINT) {
        if (m_vFuelPercent3 > CRITICAL_LOW) {
            if (m_vFuelPercent1 > (m_vFuelPercent3 + 100 - TANK_3_POINT)) {
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
    if (m_vFuelPercent3 < CRITICAL_LOW) {
        if (m_vFuelPercent2 > TANK_2_POINT) {
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
    if (m_vFuelPercent3 < CRITICAL_LOW) {
        if (m_vFuelPercent1 > CRITICAL_LOW) {
            if (m_vFuelPercent2 > CRITICAL_LOW) {
                if (m_vFuelPercent1 > m_vFuelPercent2) {
                    turn_on_pump_1();
                } else {
                    turn_on_pump_2();
                }
            } else {
                turn_on_pump_1();
            }
        } else if (m_vFuelPercent2 > CRITICAL_LOW) {
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
    uint32_t current_time = time_ms();

    M_WARN_ANSWER1::publish((m_timeAns1 + TIME_SA * 1000 < current_time) ? 1u : 0u);
    M_WARN_ANSWER2::publish((m_timeAns2 + TIME_SA * 1000 < current_time) ? 1u : 0u);
    M_WARN_ANSWER3::publish((m_timeAns3 + TIME_SA * 1000 < current_time) ? 1u : 0u);
}

void calc_flowrate()
{
    if ((bool) m_ignition::value()) {
        ignitionTotalTime += FUEL_TASK_PERIOD;
        float flowrate = (100.0f - M_FUEL_P::value()) / ((float) ignitionTotalTime / 3600000.0f);
        m_flowrate::publish(flowrate);
    }
}

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    if (size == PACK_SIZE) {
        if ((data[0] == 0x3E) && (data[2] == 0x06)) {
            if (data[8] == calcCRC(data, 8)) {
                if (data[1] == ADR_SENS1) {
                    m_timeAns1 = time_ms();
                    m_vFuelPercent1 = (float) (data[4] | (data[5] << 8)) / 10.0f;
                    M_FUEL_V1::publish(m_vFuelPercent1 * V_MAX1 / 100.0f);
                } else if (data[1] == ADR_SENS2) {
                    m_timeAns2 = time_ms();
                    m_vFuelPercent2 = (float) (data[4] | (data[5] << 8)) / 10.0f;
                    M_FUEL_V2::publish(m_vFuelPercent2 * V_MAX2 / 100.0f);
                } else if (data[1] == ADR_SENS3) {
                    m_timeAns3 = time_ms();
                    m_vFuelPercent3 = (float) (data[4] | (data[5] << 8)) / 10.0f;
                    M_FUEL_V3::publish(m_vFuelPercent3 * V_MAX3 / 100.0f);
                }
            }
        }
    }
}

EXPORT void on_task()
{
#ifdef DEBUG
    // Debug fuel simulation code
    float ctr_thr = m_thr::value();
    if (ctr_thr < 0.01f) {
        ctr_thr = 0.01f;
    }

    if (m_pump1::value() != 0) {
        float new_val = M_FUEL_V1::value() - 0.05f * ctr_thr;
        if (new_val < 0.1f) {
            new_val = 0.1f;
        }
        M_FUEL_V1::publish(new_val);
    }

    if (m_pump2::value() != 0) {
        float new_val = M_FUEL_V2::value() - 0.05f * ctr_thr;
        if (new_val < 0.1f) {
            new_val = 0.1f;
        }
        M_FUEL_V2::publish(new_val);
    }

    if (m_pump3::value() != 0) {
        float new_val = M_FUEL_V3::value() - 0.05f * ctr_thr;
        if (new_val < 0.1f) {
            new_val = 0.1f;
        }
        M_FUEL_V3::publish(new_val);
    }

    if (m_vFuelPercent3 < 100.0f && (m_pump1::value() != 0 || m_pump2::value() != 0 || m_pump3::value() != 0)) {
        float new_val = M_FUEL_V3::value() + 0.01f * ctr_thr;
        M_FUEL_V3::publish(new_val);
    }
#endif

    check_answer_time();
    calc_flowrate();

    float fl1 = M_FUEL_V1::value();
    float fl2 = M_FUEL_V2::value();
    float fl3 = M_FUEL_V3::value();
    M_FUEL_L::publish(fl1 + fl2 + fl3);

#ifdef DEBUG
    m_vFuelPercent1 = fl1 * 100 / V_MAX1; //for debug only
    m_vFuelPercent2 = fl2 * 100 / V_MAX2; //for debug only
    m_vFuelPercent3 = fl3 * 100 / V_MAX3; //for debug only
#endif

    M_FUEL_P::publish((m_vFuelPercent1 + m_vFuelPercent2 + m_vFuelPercent3) / 3);

    ignition = (bool) m_ignition::value();
    algorithm = (bool) m_algorithm::value();
    ers = (bool) m_ers::value();

    if (ers) {
        turn_off_all_pumps();
    } else {
        if (ignition != ignitionOld) {
            ignitionOld = ignition;
            if (!ignition) {
                turn_off_all_pumps();
            }
        }

        if (algorithm != algorithmOld) {
            algorithmOld = algorithm;
            if (algorithm) {
                printf("manual fuel ON");

            } else {
                turn_off_all_pumps();
                printf("manual fuel OFF");
            }
        }

        if (!algorithm && ignition) {
            fuel_auto_control();
        }
    }

    msg[0] = 0x31;
    msg[2] = 0x06;
    msg[1] = ADR_SENS1 + m_sensRequest;
    msg[3] = calcCRC(msg, 3);
    send(port_id, msg, 4, true);

    m_sensRequest = (m_sensRequest + 1) % 3;
}

int main()
{
    m_pump1();
    m_pump2();
    m_pump3();
    m_ignition();
    m_ers();
    m_algorithm();
    M_FUEL_P();
    M_FUEL_V1();
    M_FUEL_V2();
    M_FUEL_V3();
    m_thr();

    m_pump1::publish(0u);
    m_pump2::publish(0u);
    m_pump3::publish(0u);
    m_algorithm::publish(0u); //0 - auto control, 1 - manual

#ifdef DEBUG
    M_FUEL_V1::publish(V_MAX1);
    M_FUEL_V2::publish(V_MAX2);
    M_FUEL_V3::publish(V_MAX3);
#endif

    algorithmOld = false;
    ignitionOld = false;

    receive(port_id, "on_serial");

    printf("fuel stage 1");
    task("on_task", FUEL_TASK_PERIOD);

    return 0;
}
