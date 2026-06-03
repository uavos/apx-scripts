#include <apx.h>

const uint8_t PORT_ID{8};
const uint8_t PACK_SIZE_CAN{12};
const uint8_t TASK_ECU_MS{50};

const uint16_t CAN_BASE_ADDR{1520};
const uint8_t OFFSET_CAN_ID_CURRENT_THROTTLE{71};
const uint16_t CAN_ID_CURRENT_THROTTLE = CAN_BASE_ADDR + OFFSET_CAN_ID_CURRENT_THROTTLE;

//u2 u3 u4 u10 are already used in adc & gpio
//ECU values
using m_mat = Mandala<mandala::est::env::usrf::f7>;
using m_clt = Mandala<mandala::est::env::usrf::f8>;
using m_baro = Mandala<mandala::est::env::usrf::f3>;
using m_map = Mandala<mandala::est::env::usrf::f4>;
using m_ego = Mandala<mandala::est::env::usrf::f9>;
using m_tps = Mandala<mandala::est::env::usrf::f5>;
using m_vbat = Mandala<mandala::est::env::usrf::f6>;

using m_pw1 = Mandala<mandala::est::env::usrw::w1>;
using m_pw2 = Mandala<mandala::est::env::usrw::w2>;
using m_rpm = Mandala<mandala::est::env::usrw::w3>;
using m_egt_1 = Mandala<mandala::est::env::usrw::w4>;
using m_egt_2 = Mandala<mandala::est::env::usrw::w5>;
using m_egt_3 = Mandala<mandala::est::env::usrw::w6>;
using m_egt_4 = Mandala<mandala::est::env::usrw::w7>;

using m_eng_ctr = Mandala<mandala::ctr::nav::eng::thr>;
using m_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;

int main()
{
    schedule_periodic(task("on_ecu"), TASK_ECU_MS);

    m_pwr_ign(); //subscribe
    m_eng_ctr();

    receive(PORT_ID, "on_serial");
}

void setThrottleIgn()
{
    uint8_t data[13];
    data[0] = CAN_ID_CURRENT_THROTTLE & 0xFF;
    data[1] = (CAN_ID_CURRENT_THROTTLE >> 8) & 0xFF;
    data[2] = (CAN_ID_CURRENT_THROTTLE >> 16) & 0xFF;
    data[3] = (CAN_ID_CURRENT_THROTTLE >> 24) & 0xFF;

    float ch_throttle = m_eng_ctr::value(); // 0 ... +1.0
    //printf("thr:%.2f", ch_throttle);
    memcpy(&data[4], &ch_throttle, 4); // copy float bytes to data

    uint8_t power_eng = (bool) m_pwr_ign::value(); //  0 / 1
    data[8] = power_eng;
    send(PORT_ID, data, 9, true);
}

EXPORT void on_ecu()
{
    setThrottleIgn();
}

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    //printf("%d", size);
    if (size != PACK_SIZE_CAN) {
        //return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag
    //printf("can_id:%d\n", can_id - 1520);

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
        // baro, map, mat, clt
    case CAN_BASE_ADDR + 2: {
        float baro = ((int16_t) data[4] << 8 | data[5]) / 10.f;    // kPa
        float map = ((int16_t) data[6] << 8 | data[7]) / 10.f;     // kPa
        float mat_F = ((int16_t) data[8] << 8 | data[9]) / 10.f;   // deg F
        float clt_F = ((int16_t) data[10] << 8 | data[11]) / 10.f; // deg F

        float mat_C = (mat_F - 32.0f) * 5.0f / 9.0f;
        float clt_C = (clt_F - 32.0f) * 5.0f / 9.0f;

        m_mat::publish(mat_C); // manifold air temperature [deg C]
        m_clt::publish(clt_C); // Coolant temperature [deg C]
        m_baro::publish(baro); // Barometric pressure [kPa]
        m_map::publish(map);   // Manifold absolute pressure [kPa]
        break;
    }

    case CAN_BASE_ADDR + 15: {
        // oil_press.not used
        break;
    }

    // egt
    case CAN_BASE_ADDR + 22: {
        //printf("egt");
        float egt[4];
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t idx = i * 2 + 4;
            uint16_t raw_egt = (uint16_t) (data[idx] << 8) + data[idx + 1]; // EGT1 deg F s
            egt[i] = float(raw_egt);
        }

        m_egt_1::publish(egt[0] / 10.f); //EGT1
        m_egt_2::publish(egt[1] / 10.f); //EGT2
        m_egt_3::publish(egt[2] / 10.f); //EGT3
        m_egt_4::publish(egt[3] / 10.f); //EGT4
        break;
    }

    //pw1 pw2 rpm
    case CAN_BASE_ADDR + 0: {
        //uint16_t seconds = data[4] << 8 | data[5];
        uint16_t pw1 = ((uint16_t) data[6] << 8 | data[7]); //us
        uint16_t pw2 = ((uint16_t) data[8] << 8 | data[9]); //us
        uint16_t rpm = data[10] << 8 | data[11];

        m_pw1::publish((uint32_t) pw1 / 1000); // pulse width 1 [ms]
        m_pw2::publish((uint32_t) pw2 / 1000); // pulse width 2 [ms]
        m_rpm::publish((uint32_t) rpm);        // engine rpm

        break;
    }

    //tps, Vbat
    case CAN_BASE_ADDR + 3: {
        float tps = ((int16_t) (data[4] << 8) | data[5]) / 10.f; // %
        float vbat = ((int16_t) data[6] << 8 | data[7]) / 10.f;  // V
        float ego = ((int16_t) (data[8] << 8) | data[9]) / 10.f; // lambda
        //printf("ego: %.2f", ego);

        m_tps::publish(tps);   // Throttle position [%]
        m_vbat::publish(vbat); // Battery voltage [V]
        m_ego::publish(ego);   // Exhaust gas oxygen sensor [lambda]
        break;
    }
    //??
    case CAN_BASE_ADDR + 72: {
        //not used
        break;
    }
    }
}
