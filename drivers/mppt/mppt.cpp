#include <apx.h>

constexpr const port_id_t PORT_ID_CAN_AUX{20};

constexpr const uint8_t TASK_MAIN_MS{50}; //20Hz

constexpr const uint8_t PACK_CAN_SIZE{12};

constexpr const uint16_t SP_ID{512};
constexpr const uint16_t SP_PACK1{SP_ID + 1};
constexpr const uint16_t SP_PACK2{SP_ID + 2};
constexpr const uint16_t SP_POWER_ON{SP_ID + 6};

constexpr const uint16_t SP_CMD_POWER_ON{SP_ID + 10};

#pragma pack(1)
struct SP
{
    struct
    {
        float vin;
        float vout;
        float temp;
        uint8_t status;
    } msg1;

    struct
    {
        float cin;
        float cout;
    } msg2;
};
#pragma pack()

SP _sp = {};
bool sp_state{false};

using m_sp_vin = Mandala<mandala::est::env::usrf::f1>;
using m_sp_vout = Mandala<mandala::est::env::usrf::f2>;
using m_sp_temp = Mandala<mandala::est::env::usrf::f3>;
using m_sp_status = Mandala<mandala::est::env::usrf::f4>;
using m_sp_cin = Mandala<mandala::est::env::usrf::f5>;
using m_sp_cout = Mandala<mandala::est::env::usrf::f6>;

using m_sp_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>; //sp

int main()
{
    m_sp_pwr_ign();

    task("on_main", TASK_MAIN_MS); //20 Hz

    task("sp"); //GCS with terminal command `vmexec("sp")`

    receive(PORT_ID_CAN_AUX, "can_aux_handler");

    printf("MPPT script ready...\n");
}

EXPORT void sendCmdToCan(const uint32_t &can_id, const uint8_t *data, const uint8_t &size)
{
    uint8_t msg[PACK_CAN_SIZE] = {};

    msg[0] = (uint8_t) can_id;         //ID_0_7
    msg[1] = (uint8_t) (can_id >> 8);  //ID_8_15
    msg[2] = (uint8_t) (can_id >> 16); //ID_16_23
    msg[3] = (uint8_t) (can_id >> 24); //ID_24_31

    if (can_id > 0x7FF)
        msg[3] = msg[3] | 0x80;

    for (uint8_t i = 0; i < size; i++) {
        msg[4 + i] = data[i];
    }

    send(PORT_ID_CAN_AUX, msg, 4 + size, false);
    send(PORT_ID_CAN_AUX + 1, msg, 4 + size, true);
}

void sp_cmd_power_on(const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_POWER_ON, msg, 1);
}

EXPORT void on_main()
{
    bool pwr_ign = (bool) m_sp_pwr_ign::value();
    if (sp_state != pwr_ign) {
        sp_cmd_power_on(pwr_ign);
        sp_state = pwr_ign;
    }

    //save data
    m_sp_vin::publish((float) _sp.msg1.vin);
    m_sp_vout::publish((float) _sp.msg1.vout);
    m_sp_temp::publish((float) _sp.msg1.temp);
    m_sp_status::publish((float) _sp.msg1.status);
    m_sp_cin::publish((float) _sp.msg2.cin);
    m_sp_cout::publish((float) _sp.msg2.cout);
}

int16_t unpackInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t) (data[index] | (data[index + 1] << 8));
}

void processSP(const uint32_t &can_id, const uint8_t *data)
{
    auto *sp = &_sp;

    switch (can_id) {
    case SP_PACK1: {
        sp->msg1.vin = (float) unpackInt16(data, 0) / 100.f;
        sp->msg1.vout = (float) unpackInt16(data, 2) / 100.f;
        sp->msg1.temp = (float) unpackInt16(data, 4) / 100.f;
        sp->msg1.status = data[6];
        break;
    }
    case SP_PACK2: {
        memcpy(&(sp->msg2.cin), data, 8);
        break;
    }
    }
}

EXPORT void can_aux_handler(const uint8_t *data, size_t size)
{
    if (size > PACK_CAN_SIZE) {
        return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag

    uint8_t can_data[8] = {};
    const uint8_t cnt = (uint8_t) (size - 4);
    for (uint8_t i = 0; i < cnt; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    //printf("can_id:%x", can_id);

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case SP_PACK1:
    case SP_PACK2: {
        processSP(can_id, can_data);
        break;
    }
    case SP_POWER_ON: {
        printf("SP pwr:%d\n", can_data[0]);
        break;
    }
    }
}

EXPORT void sp()
{
    printf("SP status:\n");
    printf("vin: %.2f", _sp.msg1.vin);
    printf("vout: %.2f", _sp.msg1.vout);
    printf("temp: %.1f", _sp.msg1.temp);
    printf("status: %u", _sp.msg1.status);
    printf("cin: %.2f", _sp.msg2.cin);
    printf("cout: %.2f", _sp.msg2.cout);
}
