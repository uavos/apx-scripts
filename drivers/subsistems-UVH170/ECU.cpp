#include <apx.h>

const uint8_t PORT_ID{1};
const uint8_t PACK_SIZE_CAN{12};
const uint8_t TASK_ECU_MS{200};

const uint16_t CAN_BASE_ADDR{1520};
const uint8_t OFFSET_CAN_ID_CURRENT_THROTTLE{71};
const uint16_t CAN_ID_CURRENT_THROTTLE = CAN_BASE_ADDR + OFFSET_CAN_ID_CURRENT_THROTTLE;

//u2 u3 u4 u10 are already used in adc & gpio
using m_oil_temp = Mandala<mandala::est::env::usr::u13>;
using m_cool_temp = Mandala<mandala::est::env::usr::u14>;
using m_oil_press = Mandala<mandala::est::env::usr::u15>;
using m_egt_1 = Mandala<mandala::est::env::usrc::c1>;
using m_egt_2 = Mandala<mandala::est::env::usrc::c2>;
using m_egt_3 = Mandala<mandala::est::env::usrc::c3>;
using m_egt_4 = Mandala<mandala::est::env::usrc::c4>;

using m_eng_ctr = Mandala<mandala::ctr::nav::eng::thr>;

int main()
{
    schedule_periodic(task("on_ecu"), TASK_ECU_MS);

    //m_pwr_ign(); //subscribe
    //m_sw_starter();
    m_eng_ctr();

    receive(PORT_ID, "on_serial");
}

void setThrottleMS()
{
    uint8_t data[13];
    data[0] = CAN_ID_CURRENT_THROTTLE & 0xFF;
    data[1] = (CAN_ID_CURRENT_THROTTLE >> 8) & 0xFF;
    data[2] = (CAN_ID_CURRENT_THROTTLE >> 16) & 0xFF;
    data[3] = (CAN_ID_CURRENT_THROTTLE >> 24) & 0xFF;
    data[4] = 5;

    float ch_throttle = m_eng_ctr::value();
    //printf("thr:%f/n", ch_throttle);
    memcpy(&data[5], &ch_throttle, 4);
    data[9] = 1;
    send(PORT_ID, data, 10, false);
}

EXPORT void on_ecu()
{
    setThrottleMS();

    //Calc LAMBDA
}

//ECUDemandControl(uint16_t((m_eng_ctr::value() * 0.9f + 0.1f) * 1000.f)); ???????


EXPORT void on_serial(const uint8_t *data, size_t size)
{
    //printf("%d", size);
    if (size != PACK_SIZE_CAN) {
        //return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag
    //printf("can_id:%x\n", can_id);

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }


    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case CAN_BASE_ADDR + 2: {
        //printf("temps");
        float param;
        int16_t raw;

        raw = (int16_t) (data[8] << 8) + data[9];
        param = float(raw);
        float oil_temp = (param / 10.0f - 32.0f) * 5.0f / 9.0f;
        m_oil_temp::publish(oil_temp); // Oil temperature [deg]

        raw = (int16_t) (data[10] << 8) + data[11];
        param = float(raw);
        float coolant_temp = (param / 10.0f - 32.0f) * 5.0f / 9.0f;
        m_cool_temp::publish(coolant_temp); // Coolant temperature [deg]
        break;
    }

    case CAN_BASE_ADDR + 15: {
        //printf("oil pressure");
        uint16_t raw = (uint16_t) ((data[4] << 8) | data[5]); // raw param
        float param = float(raw);

        //float oil_pressure = (float(param)*0.02822581)-3.27822581); //Oil pressure
        float oil_pressure = (param * 0.02083333f) - 7.02083333f;
        m_oil_press::publish(oil_pressure);
        break;
    }

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
    }
}
