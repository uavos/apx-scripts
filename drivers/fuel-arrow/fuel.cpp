//v1.2 fuel ballance script
#include <apx.h>

static constexpr port_id_t port_id{240};
const uint8_t ADR_SENS1{75};
const uint8_t ADR_SENS2{76};
const uint8_t PACK_SIZE{9};

//editable
const float V_MAX{20.0};        //liters
const float V_BLOCK_PUMP{3.0};  //%
const float V1_START_PUMP{3.0}; //%
const float V2_START_PUMP{3.0}; //%
//const float V1_TIME_PUMP {10};  //sec
//const float V2_TIME_PUMP{10};  //sec
//const float DELTA_FUEL{3.0};   //%
const float DELTA_FUEL_WARN{8.0}; //%
const uint8_t TIME_HYSTERESIS{5}; //sec
//const float TIME_INITIALIZATION{10};   //sec
const uint8_t TIME_SENSOR_ANSWER{2}; //sec

const float KOEF_FUEL_RATIO{0.47f}; //if 0.6 then 60% - fuel1; 40% - fuel2

uint32_t m_timeAns1;
uint32_t m_timeAns2;

bool m_sensRequest = false; //false - sens1; true - sens2;

uint8_t msg[PACK_SIZE];

float m_vFuel1;
float m_vFuel2;
float m_vFuelPercent1;
float m_vFuelPercent2;
float m_vFuelWarnBalance = 0;

float new_thr;

float fuelOffset;

bool m_initSecond;
bool m_ignitionOld;
bool m_algoritmOld;

uint32_t m_timeBlockPump1;
uint32_t m_timeDeltaPump1;
uint32_t m_timeDeltaWarn;
bool m_timeBlockPump1Active = 0;
bool m_timeDeltaPump1Active = 0;
bool m_timeDeltaWarnActive = 0;
bool m_timePump1Active = 0;

using m_algorithm = Mandala<mandala::ctr::env::usr::ub1>;
using m_pump1 = Mandala<mandala::ctr::env::usr::ub2>;
using m_pump2 = Mandala<mandala::ctr::env::usr::ub3>;
using M_WARN_BALANCE = Mandala<mandala::ctr::env::usr::ub4>;
using M_WARN_ANSWER1 = Mandala<mandala::ctr::env::usr::ub5>;
using M_WARN_ANSWER2 = Mandala<mandala::ctr::env::usr::ub6>;

using M_FUEL_V1 = Mandala<mandala::ctr::env::usr::u1>;
using M_FUEL_V2 = Mandala<mandala::ctr::env::usr::u2>;
using M_DELTA_PERCENT = Mandala<mandala::ctr::env::usr::u3>;
using idx_new_thr = Mandala<mandala::ctr::env::usr::u4>;
using M_FUEL_LITR = Mandala<mandala::ctr::env::usr::u5>; //в литрах

using m_ignition = Mandala<mandala::ctr::env::pwr::eng>;
using m_ers = Mandala<mandala::ctr::env::ers::launch>;

using M_FUEL_PERC = Mandala<mandala::sns::env::fuel::level>; //в процентах

using m_thr = Mandala<mandala::ctr::nav::eng::thr>;

EXPORT float LimitSignal(float signal, float min, float max)
{
    if (signal < min)
        return min;

    if (signal > max)
        return max;

    return signal;
}

EXPORT uint8_t calcCRC(const uint8_t *buf, size_t size);

int main()
{
    m_pump1();
    m_pump2();
    m_ignition();
    m_ers();
    m_algorithm();
    M_FUEL_PERC();
    M_FUEL_LITR();
    M_FUEL_V1();
    M_FUEL_V2();
    m_thr();
    idx_new_thr();
    M_DELTA_PERCENT();
    M_WARN_BALANCE();
    M_WARN_ANSWER1();
    M_WARN_ANSWER2();

    fuelOffset = KOEF_FUEL_RATIO * 100 - (100 - KOEF_FUEL_RATIO * 100);

    m_initSecond = false;
    m_ignitionOld = false;
    m_algoritmOld = false;

    m_pump1::publish(0u);
    m_pump2::publish(0u);

    receive(port_id, "on_serial");

    task("on_task", 300);

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
    return crc % 256; //is %256 needed?
}

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    if (size == PACK_SIZE) //typePack(0)[0x3E] adr(1) fmt(2) data(3) crc(8)
    {
        if ((data[0] == 0x3E) && (data[2] == 0x06)) {
            if (data[8] == calcCRC(data, 8)) {
                if (data[1] == ADR_SENS1) {
                    m_timeAns1 = time_ms();
                    m_vFuel1 = (float) (data[4] | (data[5] << 8)) / 10.0f;
                    m_vFuelPercent1 = m_vFuel1 * 100 / V_MAX;
                } else if (data[1] == ADR_SENS2) {
                    m_timeAns2 = time_ms();
                    m_vFuel2 = (float) (data[4] | (data[5] << 8)) / 10.0f;
                    m_vFuelPercent2 = m_vFuel2 * 100 / V_MAX;
                }
            }
        }
    }
}

EXPORT void algorithm()
{
    uint32_t localTime = time_ms();
    float deltaPercent = m_vFuelPercent1 - m_vFuelPercent2 - fuelOffset;
    M_DELTA_PERCENT::publish(deltaPercent);
    //check second init
    if (m_initSecond && (m_vFuelPercent2 > V2_START_PUMP)) {
        m_initSecond = false;
        return;
    }

    //start pump1
    if (m_vFuelPercent1 > V_BLOCK_PUMP && deltaPercent > V1_START_PUMP && m_pump1::value() == 0) {
        if (!m_timeDeltaPump1Active) {
            m_timeDeltaPump1Active = true;
            m_timeDeltaPump1 = localTime;
        } else if (m_timeDeltaPump1 + TIME_HYSTERESIS * 1000 < localTime) {
            m_timeDeltaPump1Active = false;
            m_timePump1Active = true;
            m_pump1::publish(1u);
        }
    } else
        m_timeDeltaPump1Active = false;

    //stop pump1
    if (m_timePump1Active && deltaPercent <= 0) {
        m_timePump1Active = false;
        m_pump1::publish(0u);
    }

    //block pump1
    if (m_vFuelPercent1 < V_BLOCK_PUMP && m_pump1::value() != 0) {
        if (!m_timeBlockPump1Active) {
            m_timeBlockPump1Active = true;
            m_timeBlockPump1 = localTime;
        } else if (m_timeBlockPump1 + TIME_HYSTERESIS * 1000 < localTime) {
            m_timeBlockPump1Active = false;
            m_timePump1Active = false;
            m_pump1::publish(0u);
        }
    } else
        m_timeBlockPump1Active = false;

    //check disbalance
    if (fabs(deltaPercent) > DELTA_FUEL_WARN && m_vFuelWarnBalance == 0) {
        if (!m_timeDeltaWarnActive) {
            m_timeDeltaWarnActive = true;
            m_timeDeltaWarn = localTime;
        } else if (m_timeDeltaWarn + TIME_HYSTERESIS * 1000 < localTime) {
            m_timeDeltaWarnActive = false;
            M_WARN_BALANCE::publish(1u);
        }
    } else if (fabs(deltaPercent) < DELTA_FUEL_WARN && m_vFuelWarnBalance != 0) {
        if (!m_timeDeltaWarnActive) {
            m_timeDeltaWarnActive = true;
            m_timeDeltaWarn = localTime;
        } else if (m_timeDeltaWarn + TIME_HYSTERESIS * 1000 < localTime) {
            m_timeDeltaWarnActive = false;
            M_WARN_BALANCE::publish(0u);
        }
    } else
        m_timeDeltaWarnActive = false;

    //check answer time from sensor

    if (m_timeAns1 + TIME_SENSOR_ANSWER * 1000 < localTime)
        M_WARN_ANSWER1::publish(1u);
    else
        M_WARN_ANSWER1::publish(0u);

    if (m_timeAns2 + TIME_SENSOR_ANSWER * 1000 < localTime)
        M_WARN_ANSWER2::publish(1u);
    else
        M_WARN_ANSWER2::publish(0u);
}

EXPORT void on_task()
{
    M_FUEL_PERC::publish((m_vFuelPercent1 + m_vFuelPercent2) / 2.0f); // проценты
    M_FUEL_LITR::publish(m_vFuel1 + m_vFuel2);                        // литры
    M_FUEL_V1::publish(m_vFuel1);
    M_FUEL_V2::publish(m_vFuel2);

    if (m_ers::value() != 0) {
        if (m_pump1::value() != 0)
            m_pump1::publish(0u);
        if (m_pump2::value() != 0)
            m_pump2::publish(0u);
    } else {
        if (m_ignition::value() != m_ignitionOld) {
            m_ignitionOld = (bool) m_ignition::value();
            if (m_ignition::value() != 0 && m_pump2::value() == 0) {
                //m_pump2::publish(1);
            } else if (m_ignition::value() == 0 && m_pump2::value() != 0) {
                m_pump2::publish(0u);
            }
        }

        if (m_algorithm::value() != m_algoritmOld) {
            m_algoritmOld = (bool) m_algorithm::value();
            if (m_algorithm::value() != 0 && m_pump1::value() != 0) {
                m_pump2::publish(0u);
            }
        }

        if (m_algorithm::value() == 0) {
            algorithm();
        }
    }

    msg[0] = 0x31;
    msg[2] = 0x06;
    if (m_sensRequest) {
        msg[1] = ADR_SENS1;
        msg[3] = calcCRC(msg, 3);
        send(port_id, msg, 4, true);
    } else {
        msg[1] = ADR_SENS2;
        msg[3] = calcCRC(msg, 3);
        send(port_id, msg, 4, true);
    }
    m_sensRequest = !m_sensRequest;

    //=============================================
    new_thr = 0.0;
    if (m_ignition::value() > 0) {
        new_thr = 0.20f;
        new_thr += m_thr::value();
    }
    new_thr = LimitSignal(new_thr, 0.0, 1.0);
    idx_new_thr::publish(new_thr);
}
