#include <apx.h>

const uint8_t PORT_ID{60};
const uint8_t PACK_SIZE_CAN{12};
const uint8_t TASK_ECU_MS{200};

const uint16_t CAN_BASE_ADDR{1520};
const uint8_t OFFSET_CAN_ID_CURRENT_THROTTLE{71};
const uint16_t CAN_ID_CURRENT_THROTTLE = CAN_BASE_ADDR + OFFSET_CAN_ID_CURRENT_THROTTLE;

using m_oil_temp = Mandala<mandala::est::env::usr::u1>;
using m_cool_temp = Mandala<mandala::est::env::usr::u2>;
using m_oil_press = Mandala<mandala::est::env::usr::u3>;
using m_egt_1 = Mandala<mandala::est::env::usr::u4>;
using m_egt_2 = Mandala<mandala::est::env::usr::u5>;
using m_egt_3 = Mandala<mandala::est::env::usr::u6>;
using m_egt_4 = Mandala<mandala::est::env::usr::u7>;

//Vesc tail
using m_vesc_tail_rpm = Mandala<mandala::est::env::usrf::f5>;
using m_vesc_tail_current = Mandala<mandala::est::env::usr::u10>;
using m_vesc_tail_duty = Mandala<mandala::est::env::usr::u8>;
using m_vesc_tail_temp_fet = Mandala<mandala::est::env::usr::u11>;
using m_vesc_tail_temp_motor = Mandala<mandala::est::env::usr::u12>;
using m_vesc_tail_curr_in = Mandala<mandala::est::env::usr::u13>;

//Vesc gen
using m_vesc_gen_rpm = Mandala<mandala::sns::env::gen::rpm>;
using m_vesc_gen_curr_in = Mandala<mandala::sns::env::gen::current>;
using m_vesc_gen_motor_temp = Mandala<mandala::sns::env::gen::temp>;
using m_vesc_gen_fet_temp = Mandala<mandala::est::env::usrc::c6>;

//Uvhpy
using m_uvhpy_status = Mandala<mandala::est::env::usrc::c4>;
using m_uvhpy_ibat_filt = Mandala<mandala::est::env::usrf::f1>;

// AGL
using m_agl = Mandala<mandala::est::env::usr::u9>;

//VESC
//-------------------------------------------------------------------------------------
#define VESC_TAIL_ID 0x24 // VESC ID 36
#define VESC_GEN_ID 0x25  // VESC ID 37
#define STATUS_MSG_1 0x09
#define STATUS_MSG_2 0x0E
#define STATUS_MSG_3 0x0F
#define STATUS_MSG_4 0x10
#define STATUS_MSG_5 0x1B
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM 3

struct VESC_CAN_Data
{
    int32_t rpm;                 //MSG1
    float current;               //MSG1
    float duty;                  //MSG1
    uint32_t apm_hours;          //MSG2
    uint32_t apm_hours_charged;  //MSG2
    uint32_t watt_hours;         //MSG3
    uint32_t watt_hours_charged; //MSG3
    float temp_fet;              //MSG4
    float temp_mot;              //MSG4
    float curr_in;               //MSG4
    float pid_pos_now;           //MSG4
    float voltage;               //MSG5
    uint32_t tacho;              //MSG5
};

VESC_CAN_Data tail_data{};
VESC_CAN_Data gen_data{};
//-------------------------------------------------------------------------------------

//UVHPU
//-------------------------------------------------------------------------------------
#define UVHPU_ID 0x80
#define UVHPU_PACK1 UVHPU_ID + 1
#define UVHPU_PACK2 UVHPU_ID + 2
#define UVHPU_PACK3 UVHPU_ID + 3
#define UVHPU_PACK4 UVHPU_ID + 4
#define UVHPU_PACK5 UVHPU_ID + 5
#define UVHPU_PACK6 UVHPU_ID + 6
#define UVHPU_PACK7 UVHPU_ID + 7

#pragma pack(1)
struct UVHPU
{
    struct
    {
        float vbat;
        float ibat;
        float imon;
    } MSG1;
    struct
    {
        float vout;
        float tbat;
        float pbat;
        float status;
    } MSG2;
    struct
    {
        float cbat;
        float ebat;
    } MSG3;
    struct
    {
        float res_bar;
        float v_res;
    } MSG4;
    struct
    {
        float ibat_filt;
        float vbat_filt;
    } MSG5;
    struct
    {
        float cbat_res;
        float ebat_res;
    } MSG6;
    struct
    {
        int16_t life_cycles;
        float cbat_mod;
    } MSG7;
};
#pragma pack()

UVHPU _uvhpu{};

// AGL
//-------------------------------------------------------------------------------------
#define AGL_CAN_ID 0x00090002

int main()
{
    schedule_periodic(task("on_ecu"), TASK_ECU_MS);

    task("uvhpu"); //GCS with terminal command `vmexec("uvhpu")`

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

    float ch_throttle = 0.0;
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

int16_t unpackInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t) (data[index] | (data[index + 1] << 8));
}

void processUVHPUackage(const uint32_t &can_id, const uint8_t *data)
{
    switch (can_id) {
    case UVHPU_PACK1: {
        _uvhpu.MSG1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&_uvhpu.MSG1.ibat, data + 2, 4);
        _uvhpu.MSG1.imon = (float) unpackInt16(data, 6) / 100.f;
        break;
    }
    case UVHPU_PACK2: {
        _uvhpu.MSG2.vout = (float) unpackInt16(data, 0) / 100.f;
        _uvhpu.MSG2.tbat = (float) unpackInt16(data, 2) / 100.f;
        _uvhpu.MSG2.pbat = (float) unpackInt16(data, 4);
        _uvhpu.MSG2.status = data[7];

        m_uvhpy_status::publish(_uvhpu.MSG2.status);
        break;
    }
    case UVHPU_PACK3: {
        memcpy(&_uvhpu.MSG3.cbat, data, 8);
        break;
    }
    case UVHPU_PACK4: {
        memcpy(&_uvhpu.MSG4.res_bar, data, 8);
        break;
    }
    case UVHPU_PACK5: {
        memcpy(&_uvhpu.MSG5.ibat_filt, data, 8);
        m_uvhpy_ibat_filt::publish(_uvhpu.MSG5.ibat_filt);
        break;
    }
    case UVHPU_PACK6: {
        memcpy(&_uvhpu.MSG6.cbat_res, data, 8);
        break;
    }
    case UVHPU_PACK7: {
        _uvhpu.MSG7.life_cycles = unpackInt16(data, 0);
        memcpy(&_uvhpu.MSG7.cbat_mod, data + 2, 4);
        break;
    }
    }
}

EXPORT void uvhpu()
{
    printf("vbat: %.2f", _uvhpu.MSG1.vbat);
    printf("ibat: %.2f", _uvhpu.MSG1.ibat);
    printf("imon: %.2f", _uvhpu.MSG1.imon);

    printf("vout: %.2f", _uvhpu.MSG2.vout);
    printf("tbat: %.2f", _uvhpu.MSG2.tbat);
    printf("pbat: %.2f", _uvhpu.MSG2.pbat);
    printf("status: %u", _uvhpu.MSG2.status);

    printf("cbat: %.2f", _uvhpu.MSG3.cbat);
    printf("ebat: %.2f", _uvhpu.MSG3.ebat);

    printf("res_bar: %.2f", _uvhpu.MSG4.res_bar);
    printf("v_res: %.2f", _uvhpu.MSG4.v_res);

    printf("ibat_filt: %.2f", _uvhpu.MSG5.ibat_filt);
    printf("vbat_filt: %.2f", _uvhpu.MSG5.vbat_filt);

    printf("cbat_res: %.2f", _uvhpu.MSG6.cbat_res);
    printf("ebat_res: %.2f", _uvhpu.MSG6.ebat_res);

    printf("life_cycles: %u", _uvhpu.MSG7.life_cycles);
    printf("cbat_mod: %.2f", _uvhpu.MSG7.cbat_mod);
}

void processVESCPackage(const uint32_t &msg_id, const uint8_t *data, VESC_CAN_Data *vesc_data)
{
    switch (msg_id) {
    case STATUS_MSG_1: {
        vesc_data->rpm = int32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        vesc_data->current = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->duty = float(int16_t((data[6] << 8) | data[7])) / 10.f;
        break;
    }
    case STATUS_MSG_2: {
        vesc_data->apm_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
                                        | data[3]);
        vesc_data->apm_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8)
                                                | data[7]);
        break;
    }
    case STATUS_MSG_3: {
        vesc_data->watt_hours = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
                                         | data[3]);
        vesc_data->watt_hours_charged = uint32_t((data[4] << 24) | (data[5] << 16) | (data[6] << 8)
                                                 | data[7]);
        break;
    }
    case STATUS_MSG_4: {
        vesc_data->temp_fet = float(int16_t((data[0] << 8) | data[1])) / 10.f;
        vesc_data->temp_mot = float(int16_t((data[2] << 8) | data[3])) / 10.f;
        vesc_data->curr_in = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->pid_pos_now = float(int16_t((data[6] << 8) | data[7])) / 50.f;
        break;
    }
    case STATUS_MSG_5: {
        vesc_data->voltage = float(int16_t((data[4] << 8) | data[5])) / 10.f;
        vesc_data->tacho = uint32_t((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        break;
    }
    }
}

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    if (size != PACK_SIZE_CAN) {
        return;
    }
    //printf("%d", size);

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag
    //printf("can_id:%d\n", can_id);

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    if (can_id == AGL_CAN_ID) {
        //printf("agl");
        uint16_t raw = (uint16_t) ((data[4] << 8) + data[5]);
        float altitude = float(raw) / 100.0f;
        if (altitude > 0.1f && altitude < 40.0f)
            m_agl::publish(altitude);
        return;
    }

    //CAN ID 0xFF
    switch (can_id & 0xFF) {
    case VESC_TAIL_ID: {
        //printf("vesc tail %u", can_id);
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &tail_data);

        m_vesc_tail_rpm::publish((float) tail_data.rpm / 11);
        m_vesc_tail_current::publish(tail_data.current);
        m_vesc_tail_duty::publish(tail_data.duty);
        m_vesc_tail_temp_fet::publish(tail_data.temp_fet);
        m_vesc_tail_temp_motor::publish(tail_data.temp_mot);
        m_vesc_tail_curr_in::publish(tail_data.curr_in);

        break;
    }
    case VESC_GEN_ID: {
        //printf("vesc gen %u", can_id);
        uint16_t msg_id = (can_id >> 8) & 0xFF;
        processVESCPackage(msg_id, can_data, &gen_data);

        m_vesc_gen_rpm::publish((float) abs(gen_data.rpm) / 21);
        m_vesc_gen_fet_temp::publish(gen_data.temp_fet);
        m_vesc_gen_motor_temp::publish(gen_data.temp_mot);
        m_vesc_gen_curr_in::publish(fabs(gen_data.curr_in));

        break;
    }
    }

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case CAN_BASE_ADDR + 2: {
        //printf("temps");
        float param;
        int16_t raw;

        raw = (int16_t) (data[9] << 8) + data[10];
        param = float(raw);
        float oil_temp = (param / 10.0f - 32.0f) * 5.0f / 9.0f;
        m_oil_temp::publish(oil_temp); // Oil temperature [deg]

        raw = (int16_t) (data[11] << 8) + data[12];
        param = float(raw);
        float coolant_temp = (param / 10.0f - 32.0f) * 5.0f / 9.0f;
        m_cool_temp::publish(coolant_temp); // Coolant temperature [deg]
    }

    case CAN_BASE_ADDR + 15: {
        //printf("oil pressure");
        int16_t raw = (int16_t) (data[5] << 8) | data[6]; // raw param
        float param = float(raw);

        //float oil_pressure = (float(param)*0.02822581)-3.27822581); //Oil pressure
        float oil_pressure = (param * 0.02083333f) - 7.02083333f;
        m_oil_press::publish(oil_pressure);
    }

    case CAN_BASE_ADDR + 22: {
        //printf("egt");
        float egt[4];
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t idx = i * 2 + 5;
            int16_t raw_egt = (int16_t) (data[idx] << 8) + data[idx + 1]; // EGT1 deg F s
            egt[i] = float(raw_egt);
            if (data[idx] & 0x80)
                egt[i] += (0xffff << 16);
        }

        m_egt_1::publish(egt[0] / 10.f); //EGT1
        m_egt_2::publish(egt[1] / 10.f); //EGT2
        m_egt_3::publish(egt[2] / 10.f); //EGT3
        m_egt_4::publish(egt[3] / 10.f); //EGT4
    }

    case UVHPU_PACK1:
    case UVHPU_PACK2:
    case UVHPU_PACK3:
    case UVHPU_PACK4:
    case UVHPU_PACK5:
    case UVHPU_PACK6:
    case UVHPU_PACK7: {
        //printf("uvhpu %x", can_id);
        processUVHPUackage(can_id, can_data);
        break;
    }
    }
}
