#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L"; //L
constexpr const uint8_t NODE_ID{3};
constexpr const uint8_t MSG1_ID{0};
constexpr const uint8_t MSG2_ID{1};
constexpr const uint8_t MSG3_ID{2};
constexpr const port_id_t PORT_ID_CAN_AUX{20}; //L=20 //CAN   1Mb
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R"; //R
constexpr const uint8_t NODE_ID{4};
constexpr const uint8_t MSG1_ID{3};
constexpr const uint8_t MSG2_ID{4};
constexpr const uint8_t MSG3_ID{5};
constexpr const port_id_t PORT_ID_CAN_AUX{70}; //R=70 //CAN   1Mb
#endif

constexpr const uint8_t NODE_WING_L_ID{1};
constexpr const uint8_t NODE_WING_R_ID{2};

constexpr const uint16_t TASK_DELAY_MS{50}; //20Hz

//--------------------------ports--------------------------
constexpr const port_id_t PORT_ID_GCU_W{40};
constexpr const port_id_t PORT_ID_GCU_R{41};
constexpr const port_id_t PORT_ID_TLM_SYNC{42};
constexpr const port_id_t PORT_ID_WING{43};
//constexpr const port_id_t PORT_ID_MHX{44};
//---------------------------------------------------------

//--------------------------size---------------------------
constexpr const uint8_t CNT_DEV{2};
constexpr const uint8_t PACK_CAN_SIZE{12};
constexpr const uint8_t PACK_WING_SIZE{13};
constexpr const uint8_t MSG7_ID{6}; //for nav-l and nav-r wing
//---------------------------------------------------------

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
constexpr const uint16_t PU_PACK1{PU_ID + 1};
constexpr const uint16_t PU_PACK2{PU_ID + 2};
constexpr const uint16_t PU_PACK3{PU_ID + 3};
constexpr const uint16_t PU_PACK4{PU_ID + 4};
constexpr const uint16_t PU_PACK5{PU_ID + 5};
constexpr const uint16_t PU_PACK6{PU_ID + 6};
constexpr const uint16_t PU_PACK7{PU_ID + 7};

constexpr const uint16_t PU_CMD_HEATER{PU_ID + 10};
//constexpr const uint16_t PU_CMD_PUMP{PU_ID + 11};
constexpr const uint16_t PU_CMD_PON{PU_ID + 12};
//constexpr const uint16_t PU_CMD_RB{PU_ID + 13};

//data
#pragma pack(1)
struct PU
{
    struct
    {
        float vbat;
        float ibat;
        float imon;
    } msg1;
    struct
    {
        float vout;
        float tbat;
        float pbat;
        uint8_t status;
    } msg2;
    struct
    {
        float cbat;
        float ebat;
    } msg3;
    struct
    {
        float res_bar;
        float v_res;
    } msg4;
    struct
    {
        float ibat_flt;
        float vbat_flt;
    } msg5;
    struct
    {
        float cbat_res;
        float ebat_res;
    } msg6;
    struct
    {
        int16_t life_cycles;
        float cbat_mod;
        float h_pwr;
    } msg7;
};
#pragma pack()

PU _pu[2] = {};

//---------------------------------------------------------

//--------------------------mc-----------------------------
constexpr const uint16_t MC_ID{256};
constexpr const uint16_t MC_SHIFT{20};
constexpr const uint16_t MC_PACK1{MC_ID + 1};
constexpr const uint16_t MC_PACK2{MC_ID + 2};
constexpr const uint16_t MC_PACK3{MC_ID + 3};

#pragma pack(1)
struct MC
{
    float vbat;
    float tbat;
    float tpcb;
    uint8_t status;
    uint16_t cell[8] = {};
    float cell_volt(const uint8_t &cell_idx) { return cell[cell_idx] / 1000.f; };
};
#pragma pack()

MC _mc[2] = {};
//---------------------------------------------------------

//--------------------------sp-----------------------------
constexpr const uint16_t SP_ID{512};
constexpr const uint16_t SP_SHIFT{20};
constexpr const uint16_t SP_PACK1{SP_ID + 1};
constexpr const uint16_t SP_PACK2{SP_ID + 2};
constexpr const uint16_t SP_POWER_ON{SP_ID + 6};
constexpr const uint16_t SP_SAVE_K{SP_ID + 7};

constexpr const uint16_t SP_CMD_POWER_ON{SP_ID + 10};
constexpr const uint16_t SP_CMD_K{SP_ID + 11};

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

SP _sp[2] = {};
bool sp_state{false};
//---------------------------------------------------------

//--------------------------vesc---------------------------
constexpr const uint8_t VESC_ID{0x24}; //36
//---------------------------------------------------------

//----------------------Response---------------------------
uint32_t t_puResponse[CNT_DEV] = {};
uint32_t t_mcResponse[CNT_DEV] = {};
uint32_t t_spResponse[CNT_DEV] = {};
uint32_t t_escResponse{};

constexpr const uint16_t DEVICE_TIMEOUT{3000};

bool responseState[12] = {}; //pu1, mc1, sp1, pu2, mc2, sp2, vesc, volz[5]

uint8_t size_error_gcu = 0;
uint8_t size_error_sync = 0;
uint8_t size_error_wing = 0;
//----------------------------------------------------------

//-------------------------Wing-----------------------------
#pragma pack(1)
struct WING_DATA
{
    uint16_t voltage[2];
    int8_t volz_temp[2];
    uint16_t volz_pos[2];
    int8_t gyro_temp;
};
#pragma pack()

WING_DATA _wing_l = {};
WING_DATA _wing_r = {};
//----------------------------------------------------------

//----------------------Telemetry---------------------------
constexpr const uint8_t TELEMETRY_SEND_DELAY_MS{40};
uint32_t t_telemetryMsg{0};
int8_t telemetrySyncCurrentMsgId{-1};

bool needSendTelemetry{false};

constexpr const uint8_t MSG1_SIZE = 1 + 49 + 1; //header + data + crc
constexpr const uint8_t MSG2_SIZE = 1 + 49 + 1; //header + data + crc
constexpr const uint8_t MSG3_SIZE = 1 + 18 + 1; //header + data + crc

constexpr const uint8_t PACK_SIZE_MAX = 64; //max pack size
uint8_t telemetrySize = 0;
uint8_t telemetryPack[PACK_SIZE_MAX] = {};

typedef struct
{
    //pack 2
    int16_t vout; //mult=0.01
    int16_t tbat; //mult=0.01
    uint8_t status;
    //pack 3
    int32_t cbat;
    int32_t ebat;
    //pack 4
    uint16_t res_bat;
    uint16_t v_res; //mult=0.01
    //pack 5
    int16_t ibat_flt; //mult=0.01
    //pack 7
    int16_t h_pwr; //mult=0.01
} __attribute__((packed)) t_uvhpu;

typedef struct
{
    //pack 1
    int16_t vbat; //mult=0.01
    int8_t tbat;
    int8_t tpcb;
    uint8_t status;
    //pack 2
    uint16_t c1; //mult=0.001
    uint16_t c2; //mult=0.001
    uint16_t c3; //mult=0.001
    uint16_t c4; //mult=0.001
    //pack 3
    uint16_t c5; //mult=0.001
    uint16_t c6; //mult=0.001
} __attribute__((packed)) t_mcell;

typedef struct
{
    //pack 1
    int16_t vin;  //mult=0.01
    int16_t vout; //mult=0.01
    int8_t temp;
    uint8_t status;
    //pack 2
    int16_t cin;  //mult=0.01
    int16_t cout; //mult=0.01
} __attribute__((packed)) t_mppt;

typedef struct
{
    int16_t crt; //mult=0.01
    int8_t dc;
    int8_t ft;
    int8_t mt;
    int16_t crtin; //mult=0.01
} __attribute__((packed)) t_vesc;

typedef struct
{
    int8_t pos;
    int8_t temp;
} __attribute__((packed)) t_volz;

/*
 * Full packet structure:
 * | 1 byte            | N bytes   | 1 byte  |
 * | 0xF0    0x0F
 * | NodeId  MessageId | data      | crc8    |
 */
typedef struct
{
    uint8_t uvhpu_online : 1;
    uint8_t mcell_online : 1;
    uint8_t mppt_online : 1;
    uint8_t unused : 5;
    t_uvhpu pu;
    t_mcell mc;
    t_mppt mppt;
} __attribute__((packed)) t_ifc_lr_msg1;

typedef struct
{
    uint8_t vesc_online : 1;
    uint8_t volz_ail_online : 1;
    uint8_t volz_elv_online : 1;
    uint8_t volz_rud_online : 1;
    uint8_t volz_pitch_online : 1;
    uint8_t unused : 3;
    t_vesc vesc;
    t_volz volz_ail;
    t_volz volz_sweep;
    t_volz volz_elv;
    t_volz volz_rud;
    int8_t wind_temp;
} __attribute__((packed)) t_ifc_lr_msg2;
//---------------------------------------------------------

//----------------------saveToMandala----------------------
constexpr const uint8_t SAVE_MANDALA_DELAY_MS{100};
//---------------------------------------------------------

//pu
using m_pu_vbat = Mandala<mandala::est::env::usr::u3>;
using m_pu_temp = Mandala<mandala::est::env::usrw::w3>;
using m_pu_pwr = Mandala<mandala::est::env::usr::u4>;
using m_pu_status = Mandala<mandala::est::env::usrc::c7>;
using m_pu_cbat_flt = Mandala<mandala::est::env::usrf::f5>;
using m_pu_vbat_flt = Mandala<mandala::est::env::usrf::f6>;
using m_pu_h_pwr = Mandala<mandala::est::env::usrc::c10>;

using m_pu_cmd_hpwr = Mandala<mandala::est::env::usrc::c11>;

//mc
using m_mc_vbat = Mandala<mandala::est::env::usrf::f7>;
using m_mc_tbat = Mandala<mandala::est::env::usrw::w4>;
using m_mc_tpcb = Mandala<mandala::est::env::usrw::w5>;
using m_mc_status = Mandala<mandala::est::env::usrc::c8>;

//sp
using m_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>;

using m_sp_vin = Mandala<mandala::est::env::usrf::f8>;
using m_sp_vout = Mandala<mandala::est::env::usrf::f9>;
using m_sp_temp = Mandala<mandala::est::env::usr::u1>;
using m_sp_status = Mandala<mandala::est::env::usrc::c9>;
using m_sp_cin = Mandala<mandala::est::env::usr::u2>;
using m_sp_cout = Mandala<mandala::est::env::usr::u9>;

using m_sp_cmd_k = Mandala<mandala::ctr::env::tune::t15>;

//nav
using m_navl_temp = Mandala<mandala::est::env::usr::u10>;
using m_navr_temp = Mandala<mandala::est::env::usr::u11>;

int main()
{
    m_pwr_ign();

    const auto now = time_ms();

    t_puResponse[0] = now;
    t_puResponse[1] = now;

    t_mcResponse[0] = now;
    t_mcResponse[1] = now;

    t_spResponse[0] = now;
    t_spResponse[1] = now;
    t_escResponse = now;

    t_telemetryMsg = now;

    //task("reset_error"); //GCS with terminal command `vmexec("reset_error")`

    //task("pu_on");  //GCS with terminal command `vmexec("pu_on")`
    //task("pu_off"); //GCS with terminal command `vmexec("pu_off")`
    //task("sp_on");  //GCS with terminal command `vmexec("sp_on")`
    //task("sp_off"); //GCS with terminal command `vmexec("sp_off")`

    task("wing_l"); //GCS with terminal command `vmexec("wing_l")`
    task("wing_r"); //GCS with terminal command `vmexec("wing_r")`
    m_pu_cmd_hpwr("on_cmd_hpwr");
    m_sp_cmd_k("on_cmd_k");

#if defined NODE_LEFT
    //task("mc_1"); //GCS with terminal command `vmexec("mc_1")`
    task("mc_2"); //GCS with terminal command `vmexec("mc_2")`

    //task("pu_1"); //GCS with terminal command `vmexec("pu_1")`
    task("pu_2"); //GCS with terminal command `vmexec("pu_2")`

    //task("sp_1"); //GCS with terminal command `vmexec("sp_1")`
    task("sp_2"); //GCS with terminal command `vmexec("sp_2")`

#endif

#if defined NODE_RIGHT
    task("mc_3"); //GCS with terminal command `vmexec("mc_3")`
    task("mc_4"); //GCS with terminal command `vmexec("mc_4")`

    task("pu_3"); //GCS with terminal command `vmexec("pu_3")`
    task("pu_4"); //GCS with terminal command `vmexec("pu_4")`

    task("sp_3"); //GCS with terminal command `vmexec("sp_3")`
    task("sp_4"); //GCS with terminal command `vmexec("sp_4")`

    task("wing_r"); //GCS with terminal command `vmexec("wing")`
#endif

    task("on_task", TASK_DELAY_MS);                 //20 Hz
    task("on_save_mandala", SAVE_MANDALA_DELAY_MS); //10 Hz

    receive(PORT_ID_CAN_AUX, "canAuxHandler");
    receive(PORT_ID_TLM_SYNC, "tlmSyncHandler");
    receive(PORT_ID_GCU_R, "gcuHandler");
    receive(PORT_ID_WING, "wingHandler");

    printf("IFC:%s Script ready...\n", txt_dev);

    return 0;
}

template<typename T>
T limit(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void checkDeviceOnline(const uint32_t &t_LastResponse, const uint8_t &idx)
{
    uint32_t now = time_ms();

    if (now - t_LastResponse > DEVICE_TIMEOUT) {
        responseState[idx] = false;
    } else {
        responseState[idx] = true;
    }
}

uint8_t calcTelemetryCRC(const uint8_t *data, uint8_t size)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = crc & 0x80 ? (uint8_t) (crc << 1) ^ 0x31 : (uint8_t) (crc << 1);
    }
    return crc;
}

int16_t unpackInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t) (data[index] | (data[index + 1] << 8));
}

void packMsg1(const uint8_t &idx)
{
    t_ifc_lr_msg1 msg = {};

    msg.uvhpu_online = responseState[idx * 3 + 0];
    msg.mcell_online = responseState[idx * 3 + 1];
    msg.mppt_online = responseState[idx * 3 + 2];

    //pu
    auto *pu = &_pu[idx];

    //pack 2
    msg.pu.vout = (int16_t) (pu->msg2.vout * 100.f);
    msg.pu.tbat = (int16_t) (pu->msg2.tbat * 100.f);
    msg.pu.status = (uint8_t) pu->msg2.status;
    //pack 3
    msg.pu.cbat = (int32_t) pu->msg3.cbat;
    msg.pu.ebat = (int32_t) (pu->msg3.ebat * 100.f);
    //pack 4
    msg.pu.res_bat = (uint16_t) (pu->msg4.res_bar * 100.f);
    msg.pu.v_res = (uint16_t) (pu->msg4.v_res * 100.f);
    //pack 5
    msg.pu.ibat_flt = (int16_t) (pu->msg5.ibat_flt * 100.f);
    //pack 7
    msg.pu.h_pwr = (int16_t) (pu->msg7.h_pwr * 100.f);

    //mc
    auto *mc = &_mc[idx];

    //pack 1
    msg.mc.vbat = int16_t(mc->vbat * 100.f);
    msg.mc.tbat = int8_t(mc->tbat);
    msg.mc.tpcb = int8_t(mc->tpcb);
    msg.mc.status = mc->status;
    //pack 2
    msg.mc.c1 = mc->cell[0];
    msg.mc.c2 = mc->cell[1];
    msg.mc.c3 = mc->cell[2];
    msg.mc.c4 = mc->cell[3];
    //pack 3
    msg.mc.c5 = mc->cell[4];
    msg.mc.c6 = mc->cell[5];

    memcpy(&telemetryPack[1], &msg, sizeof(t_ifc_lr_msg1));
}

void fillTelemetryPack()
{
    //HEADER
    telemetryPack[0] = (telemetrySyncCurrentMsgId & 0x0F) | ((NODE_ID << 4) & 0xF0);

    if (telemetrySyncCurrentMsgId == MSG1_ID) {
        telemetrySize = MSG1_SIZE;
        packMsg1(0);
    } else if (telemetrySyncCurrentMsgId == MSG2_ID) {
        telemetrySize = MSG2_SIZE;
        packMsg1(1);
    } else if (telemetrySyncCurrentMsgId == MSG3_ID) {
        telemetrySize = MSG3_SIZE;
    }

    //crc
    uint8_t crc = calcTelemetryCRC(telemetryPack, telemetrySize - 1);
    telemetryPack[telemetrySize - 1] = crc;
}

void sendTelemetry()
{
    send(PORT_ID_GCU_W, telemetryPack, telemetrySize, true);
    needSendTelemetry = false;
    telemetrySyncCurrentMsgId = -1;
}

void saveDataToMandala()
{
    uint8_t idx = 1;

    //pu
    m_pu_vbat::publish(_pu[idx].msg1.vbat);
    m_pu_temp::publish(_pu[idx].msg2.tbat);
    m_pu_pwr::publish(_pu[idx].msg2.pbat);
    m_pu_status::publish((uint32_t) _pu[idx].msg2.status);
    m_pu_cbat_flt::publish(_pu[idx].msg5.ibat_flt);
    m_pu_vbat_flt::publish(_pu[idx].msg5.vbat_flt);
    m_pu_h_pwr::publish(_pu[idx].msg7.h_pwr);

    //mc
    m_mc_vbat::publish(_mc[idx].vbat);
    m_mc_tbat::publish(_mc[idx].tbat);
    m_mc_tpcb::publish(_mc[idx].tpcb);
    m_mc_status::publish((uint32_t) _mc[idx].status);

    //sp
    m_sp_vin::publish(_sp[idx].msg1.vin);
    m_sp_vout::publish(_sp[idx].msg1.vout);
    m_sp_temp::publish(_sp[idx].msg1.temp);
    m_sp_status::publish((uint32_t) _sp[idx].msg1.status);
    m_sp_cin::publish(_sp[idx].msg2.cin);
    m_sp_cout::publish(_sp[idx].msg2.cout);

    //nav
    m_navl_temp::publish((float) _wing_l.gyro_temp);
    m_navr_temp::publish((float) _wing_r.gyro_temp);
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

void pu_cmd_power_on(const uint8_t &value)
{
    uint8_t msg[1] = {value};

    sendCmdToCan(PU_CMD_PON, msg, 1);
    sendCmdToCan(PU_CMD_PON + PU_SHIFT, msg, 1);
}

void pu_cmd_heater(const uint8_t &dev_id, const uint8_t &value)
{
    const uint16_t cmd = value * 100;

    uint8_t msg[2] = {};
    msg[0] = (uint8_t) cmd;
    msg[1] = cmd >> 8;

    sendCmdToCan(PU_CMD_HEATER + dev_id * PU_SHIFT, msg, 2);
}

void sp_cmd_power_on(const uint8_t &value)
{
    uint8_t msg[1] = {value};

    sendCmdToCan(SP_CMD_POWER_ON, msg, 1);
    sendCmdToCan(SP_CMD_POWER_ON + SP_SHIFT, msg, 1);
}

void sp_cmd_k(const float &value)
{
    uint8_t msg[4] = {};
    memcpy(msg, &value, sizeof(float));

    sendCmdToCan(SP_CMD_K, msg, 1);
    sendCmdToCan(SP_CMD_K + SP_SHIFT, msg, 4);
}

void processPU(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *pu = &_pu[idx];

    switch (can_id) {
    case PU_PACK1: {
        pu->msg1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&pu->msg1.ibat, data + 2, 4);
        pu->msg1.imon = (float) unpackInt16(data, 6) / 100.f;
        break;
    }
    case PU_PACK2: {
        pu->msg2.vout = (float) unpackInt16(data, 0) / 100.f;
        pu->msg2.tbat = (float) unpackInt16(data, 2) / 100.f;
        pu->msg2.pbat = (float) unpackInt16(data, 4);
        pu->msg2.status = data[6];
        break;
    }
    case PU_PACK3: {
        memcpy(&pu->msg3.cbat, data, 8);
        break;
    }
    case PU_PACK4: {
        memcpy(&pu->msg4.res_bar, data, 8);
        break;
    }
    case PU_PACK5: {
        memcpy(&pu->msg5.ibat_flt, data, 8);
        break;
    }
    case PU_PACK6: {
        memcpy(&pu->msg6.cbat_res, data, 8);
        break;
    }
    case PU_PACK7: {
        pu->msg7.h_pwr = unpackInt16(data, 6) / 100.f;
        break;
    }
    }
}

void processMC(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *mc = &_mc[idx];

    switch (can_id) {
    case MC_PACK1: {
        mc->vbat = (float) unpackInt16(data, 0) / 100.f;
        mc->tbat = (float) unpackInt16(data, 2) / 100.f;
        mc->tpcb = (float) unpackInt16(data, 4) / 100.f;
        mc->status = data[6];
        break;
    }
    case MC_PACK2: {
        memcpy(mc->cell, data, 8);
        break;
    }
    case MC_PACK3: {
        memcpy(mc->cell + 4, data, 8);
        break;
    }
    }
}

void processSP(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *sp = &_sp[idx];

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

EXPORT void canAuxHandler(const uint8_t *data, size_t size)
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

    const uint32_t now = time_ms();

    //CAN ID 0xFF
    if ((can_id & 0xFF) == VESC_ID) {
        t_escResponse = now;
    }

    //printf("can_id:%x", can_id);

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case PU_PACK1:
    case PU_PACK2:
    case PU_PACK3:
    case PU_PACK4:
    case PU_PACK5:
    case PU_PACK6:
    case PU_PACK7: {
        t_puResponse[0] = now;
        processPU(can_id, can_data, 0);
        break;
    }
    case PU_PACK1 + 1 * PU_SHIFT:
    case PU_PACK2 + 1 * PU_SHIFT:
    case PU_PACK3 + 1 * PU_SHIFT:
    case PU_PACK4 + 1 * PU_SHIFT:
    case PU_PACK5 + 1 * PU_SHIFT:
    case PU_PACK6 + 1 * PU_SHIFT:
    case PU_PACK7 + 1 * PU_SHIFT: {
        t_puResponse[1] = now;
        can_id -= 1 * PU_SHIFT;
        processPU(can_id, can_data, 1);
        break;
    }
    case MC_PACK1:
    case MC_PACK2:
    case MC_PACK3: {
        t_mcResponse[0] = now;
        processMC(can_id, can_data, 0);
        break;
    }
    case MC_PACK1 + 1 * MC_SHIFT:
    case MC_PACK2 + 1 * MC_SHIFT:
    case MC_PACK3 + 1 * MC_SHIFT: {
        t_mcResponse[1] = now;
        can_id -= 1 * MC_SHIFT;
        processMC(can_id, can_data, 1);
        break;
    }
    case SP_PACK1:
    case SP_PACK2: {
        t_spResponse[0] = now;
        processSP(can_id, can_data, 0);
        break;
    }
    case SP_PACK1 + 1 * SP_SHIFT:
    case SP_PACK2 + 1 * SP_SHIFT: {
        t_spResponse[1] = now;
        can_id -= 1 * SP_SHIFT;
        processSP(can_id, can_data, 1);
        break;
    }
    case SP_POWER_ON:
    case SP_POWER_ON + 1 * SP_SHIFT: {
        //uint32_t n_sp = (can_id - SP_POWER_ON) / SP_SHIFT + 1;
        //printf("SP-%s", txt_dev);
        //printf("%d ", n_sp);
        printf("SP pwr:%d\n", can_data[0]);
        break;
    }
    case SP_SAVE_K:
    case SP_SAVE_K + 1 * SP_SHIFT: {
        float cmd{};
        memcpy(&cmd, can_data, sizeof(float));
        printf("SP-k:%.2f\n", cmd);
    }
    }
}

EXPORT void tlmSyncHandler(const uint8_t *data, size_t size)
{
    if (size != 2 && size_error_sync++ < 3) {
        printf("IFC-%s, wrong telemetry sync packet...\n", txt_dev);
        return;
    }

    if (data[0] == NODE_ID) {
        uint8_t msg_id = data[1];
        if (msg_id == MSG1_ID || msg_id == MSG2_ID || msg_id == MSG3_ID) {
            telemetrySyncCurrentMsgId = (int8_t) msg_id;
            needSendTelemetry = true;
            t_telemetryMsg = time_ms();
            fillTelemetryPack();
        }
    }
}

EXPORT void gcuHandler(const uint8_t *data, size_t size)
{
    (void) data;
    (void) size;
}

EXPORT void wingHandler(const uint8_t *data, size_t size)
{
    if (size != PACK_WING_SIZE && size_error_wing++ < 3) {
        printf("IFC-%s, wrong telemetry wing packet...\n", txt_dev);
        return;
    }

    //TODO CRC
    if (data[0] == (MSG7_ID | ((NODE_WING_L_ID << 4) & 0xF0))) {
        memcpy(&_wing_l, &data[1], sizeof(WING_DATA));
    }

    if (data[0] == (MSG7_ID | ((NODE_WING_R_ID << 4) & 0xF0))) {
        memcpy(&_wing_r, &data[1], sizeof(WING_DATA));
    }
}

EXPORT void on_task()
{
    checkDeviceOnline(t_puResponse[0], 0);
    checkDeviceOnline(t_mcResponse[0], 1);
    checkDeviceOnline(t_spResponse[0], 2);

    checkDeviceOnline(t_puResponse[1], 3);
    checkDeviceOnline(t_mcResponse[1], 4);
    checkDeviceOnline(t_spResponse[1], 5);

    checkDeviceOnline(t_escResponse, 6);

    bool pwr_ign = (bool) m_pwr_ign::value();
    if (sp_state != pwr_ign) {
        sp_cmd_power_on(pwr_ign);
        sp_state = pwr_ign;
    }

    uint32_t now = time_ms();

    if (now - t_telemetryMsg > TELEMETRY_SEND_DELAY_MS && needSendTelemetry) {
        sendTelemetry();
    }
}

EXPORT void on_save_mandala()
{
    saveDataToMandala();
}

EXPORT void on_cmd_hpwr()
{
    constexpr const uint8_t PU_MIN_HEATER{0};
    constexpr const uint8_t PU_MAX_HEATER{30};
    uint8_t cmd = limit((uint8_t) m_pu_cmd_hpwr::value(), PU_MIN_HEATER, PU_MAX_HEATER);
    printf("cmd_hpwr:%u", cmd);
    pu_cmd_heater(1, cmd);
}

EXPORT void on_cmd_k()
{
    constexpr const float PU_MIN_CMD_K{0.7f};
    constexpr const float PU_MAX_CMD_K{0.95f};

    float cmd = limit((float) m_sp_cmd_k::value(), PU_MIN_CMD_K, PU_MAX_CMD_K);
    printf("cmd_k:%.2f", cmd);
    sp_cmd_k(cmd);
}

EXPORT void reset_error()
{
    //printf("IFC-%s: sync_size(%u)\n", txt_dev, size_error_sync);
    //printf("IFC-%s: gcu_size(%u)\n", txt_dev, size_error_gcu);
    //printf("IFC-%s: wing_size(%u)\n", txt_dev, size_error_wing);

    size_error_sync = 0;
    size_error_gcu = 0;
    size_error_wing = 0;
}

void print_mc(const uint8_t &idx)
{
    printf("v_bat: %.2f", _mc[idx].vbat);
    printf("t_bat: %.2f", _mc[idx].tbat);
    printf("t_pcb: %.2f", _mc[idx].tpcb);
    printf("state: %u", _mc[idx].status);
    printf("C[1]: %.2f", _mc[idx].cell_volt(0));
    printf("C[2]: %.2f", _mc[idx].cell_volt(1));
    printf("C[3]: %.2f", _mc[idx].cell_volt(2));
    printf("C[4]: %.2f", _mc[idx].cell_volt(3));
    printf("C[5]: %.2f", _mc[idx].cell_volt(4));
    printf("C[6]: %.2f", _mc[idx].cell_volt(5));
}

EXPORT void mc_1()
{
    printf("IFC-%s, mc_1...\n", txt_dev);
    print_mc(0);
}

EXPORT void mc_2()
{
    printf("IFC-%s, mc_2...\n", txt_dev);
    print_mc(1);
}

EXPORT void mc_3()
{
    printf("IFC-%s, mc_3...\n", txt_dev);
    print_mc(0);
}

EXPORT void mc_4()
{
    printf("IFC-%s, mc_4...\n", txt_dev);
    print_mc(1);
}

void print_pu(const uint8_t &idx)
{
    printf("vbat: %.2f", _pu[idx].msg1.vbat);
    printf("ibat: %.2f", _pu[idx].msg1.ibat);
    printf("imon: %.2f", _pu[idx].msg1.imon);
    printf("vout: %.2f", _pu[idx].msg2.vout);
    printf("tbat: %.2f", _pu[idx].msg2.tbat);
    printf("pbat: %.2f", _pu[idx].msg2.pbat);
    printf("status: %u", _pu[idx].msg2.status);
    //printf("cbat: %.1f", _pu[idx].msg3.cbat);
    //printf("ebat: %.1f", _pu[idx].msg3.ebat);
    printf("ibat_filt: %.2f", _pu[idx].msg5.ibat_flt);
    printf("vbat_filt: %.2f", _pu[idx].msg5.vbat_flt);
    printf("h_pwr: %.1f", _pu[idx].msg7.h_pwr);
}

EXPORT void pu_1()
{
    printf("IFC-%s, pu_1...\n", txt_dev);
    print_pu(0);
}

EXPORT void pu_2()
{
    printf("IFC-%s, pu_2...\n", txt_dev);
    print_pu(1);
}

EXPORT void pu_3()
{
    printf("IFC-%s, pu_3...\n", txt_dev);
    print_pu(0);
}

EXPORT void pu_4()
{
    printf("IFC-%s, pu_4...\n", txt_dev);
    print_pu(1);
}

void print_sp(const uint8_t &idx)
{
    printf("vin: %.2f", _sp[idx].msg1.vin);
    printf("vout: %.2f", _sp[idx].msg1.vout);
    printf("temp: %.1f", _sp[idx].msg1.temp);
    printf("status: %u", _sp[idx].msg1.status);
    printf("cin: %.2f", _sp[idx].msg2.cin);
    printf("cout: %.2f", _sp[idx].msg2.cout);
}

EXPORT void sp_1()
{
    printf("IFC-%s, sp_1...\n", txt_dev);
    print_sp(0);
}

EXPORT void sp_2()
{
    printf("IFC-%s, sp_2...\n", txt_dev);
    print_sp(1);
}

EXPORT void sp_3()
{
    printf("IFC-%s, sp_3...\n", txt_dev);
    print_sp(0);
}

EXPORT void sp_4()
{
    printf("IFC-%s, sp_4...\n", txt_dev);
    print_sp(1);
}

EXPORT void wing_l()
{
    printf("IFC-%s, wing...\n", txt_dev);
    printf("volt1: %.2f", _wing_l.voltage[0] / 100.f);
    printf("volt2: %.2f", _wing_l.voltage[1] / 100.f);
    printf("temp1: %d", _wing_l.volz_temp[0]);
    printf("temp2: %d", _wing_l.volz_temp[1]);
    printf("pos1: %u", _wing_l.volz_pos[0]);
    printf("pos2: %u", _wing_l.volz_pos[1]);
    printf("temp: %d", _wing_l.gyro_temp);
}

EXPORT void wing_r()
{
    printf("IFC-%s, wing...\n", txt_dev);
    printf("volt1: %.2f", _wing_r.voltage[0] / 100.f);
    printf("volt2: %.2f", _wing_r.voltage[1] / 100.f);
    printf("temp1: %d", _wing_r.volz_temp[0]);
    printf("temp2: %d", _wing_r.volz_temp[1]);
    printf("pos1: %u", _wing_r.volz_pos[0]);
    printf("pos2: %u", _wing_r.volz_pos[1]);
    printf("temp: %d", _wing_r.gyro_temp);
}

EXPORT void pu_on()
{
    printf("IFC-%s, pu_on...\n", txt_dev);
    pu_cmd_power_on(1u);
}

EXPORT void pu_off()
{
    printf("IFC-%s, pu_off...\n", txt_dev);
    pu_cmd_power_on(0u);
}

EXPORT void sp_on()
{
    printf("IFC-%s, sp_on...\n", txt_dev);
    sp_cmd_power_on(1u);
}

EXPORT void sp_off()
{
    printf("IFC-%s, sp_off...\n", txt_dev);
    sp_cmd_power_on(0u);
}
