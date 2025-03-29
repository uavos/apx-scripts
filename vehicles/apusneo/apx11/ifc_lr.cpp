#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L";           //L
constexpr const uint8_t NODE_ID{3};            //L=1
constexpr const uint8_t MSG1_ID{0};            //L=0
constexpr const uint8_t MSG2_ID{1};            //L=1
constexpr const uint8_t MSG3_ID{2};            //L=2
constexpr const uint8_t NODE_WING_ID{1};       //L=1
constexpr const port_id_t PORT_ID_CAN_AUX{20}; //L=20 //CAN   1Mb
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R";           //R
constexpr const uint8_t NODE_ID{4};            //R=2
constexpr const uint8_t MSG1_ID{3};            //R=3
constexpr const uint8_t MSG2_ID{4};            //R=4
constexpr const uint8_t MSG3_ID{5};            //R=5
constexpr const uint8_t NODE_WING_ID{2};       //R=2
constexpr const port_id_t PORT_ID_CAN_AUX{70}; //R=70 //CAN   1Mb
#endif

constexpr const uint16_t TASK_DELAY_MS{20}; //50Hz

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
constexpr const uint8_t PACK_WING_SIZE{9};
constexpr const uint8_t MSG7_ID{6}; //for nav-l and nav-r wing
//---------------------------------------------------------

//--------------------------uvhpu--------------------------
constexpr const uint16_t UVHPU_ID{128};
constexpr const uint16_t UVHPU_SHIFT{20};
constexpr const uint16_t UVHPU_PACK1{UVHPU_ID + 1};
constexpr const uint16_t UVHPU_PACK2{UVHPU_ID + 2};
constexpr const uint16_t UVHPU_PACK3{UVHPU_ID + 3};
constexpr const uint16_t UVHPU_PACK4{UVHPU_ID + 4};
constexpr const uint16_t UVHPU_PACK5{UVHPU_ID + 5};
constexpr const uint16_t UVHPU_PACK6{UVHPU_ID + 6};
constexpr const uint16_t UVHPU_PACK7{UVHPU_ID + 7};

//constexpr const uint16_t UVHPU_CMD_HEATER{UVHPU_ID + 10};
//constexpr const uint16_t UVHPU_CMD_PUMP{UVHPU_ID + 11};
constexpr const uint16_t UVHPU_CMD_PON{UVHPU_ID + 12};
//constexpr const uint16_t UVHPU_CMD_RB{UVHPU_ID + 13};

//data
#pragma pack(1)
struct UVHPU
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
        int16_t h_pwr;
    } msg7;
};
#pragma pack()

UVHPU _uvhpu[2] = {};

//---------------------------------------------------------

//--------------------------mcell--------------------------
constexpr const uint16_t MCELL_ID{256};
constexpr const uint16_t MCELL_SHIFT{20};
constexpr const uint16_t MCELL_PACK1{MCELL_ID + 1};
constexpr const uint16_t MCELL_PACK2{MCELL_ID + 2};
constexpr const uint16_t MCELL_PACK3{MCELL_ID + 3};

#pragma pack(1)
struct MCELL
{
    float vbat;
    float tbat;
    float tpcb;
    uint8_t status;
    uint16_t cell[8] = {};
    float cell_volt(const uint8_t &cell_idx) { return cell[cell_idx] / 1000.f; };
};
#pragma pack()

MCELL _mcell[2] = {};
//---------------------------------------------------------

//--------------------------vesc---------------------------
constexpr const uint8_t VESC_ID{0x24}; //36
//---------------------------------------------------------

//----------------------Response---------------------------
uint32_t t_uvhpuResponse[CNT_DEV] = {};
uint32_t t_mcellResponse[CNT_DEV] = {};
uint32_t t_vescResponse{};

constexpr const uint16_t DEVICE_TIMEOUT{3000};

bool responseState[12] = {}; //uvhpu1, mcell1, mppt1, uvhpu2, mcell2, mppt2, vesc, volz[5]

uint8_t size_error_gcu = 0;
uint8_t size_error_sync = 0;
uint8_t size_error_wing = 0;
//----------------------------------------------------------

//-------------------------Wing-----------------------------
#pragma pack(1)
struct WING_DATA
{
    int8_t volz_temp[2];
    uint16_t volz_pos[2];
    int8_t gyro_temp;
};
#pragma pack()

WING_DATA _wing = {};
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
    t_uvhpu uvhpu;
    t_mcell mcell;
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

int main()
{
    uint32_t now = time_ms();

    t_uvhpuResponse[0] = now;
    t_uvhpuResponse[1] = now;

    t_mcellResponse[0] = now;
    t_mcellResponse[1] = now;
    t_vescResponse = now;

    t_telemetryMsg = now;

    //task("reset_error"); //GCS with terminal command `vmexec("reset_error")`
    task("pu_on");  //GCS with terminal command `vmexec("pu_on")`
    task("pu_off"); //GCS with terminal command `vmexec("pu_off")`

#if defined NODE_LEFT
    task("mcell_1"); //GCS with terminal command `vmexec("mcell_1")`
    task("mcell_2"); //GCS with terminal command `vmexec("mcell_2")`

    task("uvhpu_1"); //GCS with terminal command `vmexec("uvhpu_1")`
    task("uvhpu_2"); //GCS with terminal command `vmexec("uvhpu_2")`

    task("wing_l"); //GCS with terminal command `vmexec("wing")`
#endif

#if defined NODE_RIGHT
    task("mcell_3"); //GCS with terminal command `vmexec("mcell_3")`
    task("mcell_4"); //GCS with terminal command `vmexec("mcell_4")`

    task("uvhpu_3"); //GCS with terminal command `vmexec("uvhpu_3")`
    task("uvhpu_4"); //GCS with terminal command `vmexec("uvhpu_4")`

    task("wing_r"); //GCS with terminal command `vmexec("wing")`
#endif

    task("on_task", TASK_DELAY_MS);                 //50 Hz
    task("on_save_mandala", SAVE_MANDALA_DELAY_MS); //10 Hz

    receive(PORT_ID_CAN_AUX, "canAuxHandler");
    receive(PORT_ID_TLM_SYNC, "tlmSyncHandler");
    receive(PORT_ID_GCU_R, "gcuHandler");
    receive(PORT_ID_WING, "wingHandler");

    printf("IFC:%s Script ready...\n", txt_dev);

    return 0;
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

    //uvhpu
    auto *uvhpu = &_uvhpu[idx];

    //pack 2
    msg.uvhpu.vout = (int16_t) (uvhpu->msg2.vout * 100.f);
    msg.uvhpu.tbat = (int16_t) (uvhpu->msg2.tbat * 100.f);
    msg.uvhpu.status = (uint8_t) uvhpu->msg2.status;
    //pack 3
    msg.uvhpu.cbat = (int32_t) uvhpu->msg3.cbat;
    msg.uvhpu.ebat = (int32_t) (uvhpu->msg3.ebat * 100.f);
    //pack 4
    msg.uvhpu.res_bat = (uint16_t) (uvhpu->msg4.res_bar * 100.f);
    msg.uvhpu.v_res = (uint16_t) (uvhpu->msg4.v_res * 100.f);
    //pack 5
    msg.uvhpu.ibat_flt = (int16_t) (uvhpu->msg5.ibat_flt * 100.f);
    //pack 7
    msg.uvhpu.h_pwr = uvhpu->msg7.h_pwr;

    //mcell
    auto *mcell = &_mcell[idx];

    //pack 1
    msg.mcell.vbat = int16_t(mcell->vbat * 100.f);
    msg.mcell.tbat = int8_t(mcell->tbat);
    msg.mcell.tpcb = int8_t(mcell->tpcb);
    msg.mcell.status = mcell->status;
    //pack 2
    msg.mcell.c1 = mcell->cell[0];
    msg.mcell.c2 = mcell->cell[1];
    msg.mcell.c3 = mcell->cell[2];
    msg.mcell.c4 = mcell->cell[3];
    //pack 3
    msg.mcell.c5 = mcell->cell[4];
    msg.mcell.c6 = mcell->cell[5];

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

void saveDataToMandala() {}

EXPORT void on_task()
{
    checkDeviceOnline(t_uvhpuResponse[0], 0);
    checkDeviceOnline(t_mcellResponse[0], 1);

    checkDeviceOnline(t_uvhpuResponse[1], 3);
    checkDeviceOnline(t_mcellResponse[1], 4);

    checkDeviceOnline(t_vescResponse, 6);

    uint32_t now = time_ms();

    if (now - t_telemetryMsg > TELEMETRY_SEND_DELAY_MS && needSendTelemetry) {
        sendTelemetry();
    }
}

EXPORT void on_save_mandala()
{
    saveDataToMandala();
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
}

void processUVHPUackage(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *uvhpu = &_uvhpu[idx];

    switch (can_id) {
    case UVHPU_PACK1: {
        uvhpu->msg1.vbat = (float) unpackInt16(data, 0) / 100.f;
        memcpy(&uvhpu->msg1.ibat, data + 2, 4);
        uvhpu->msg1.imon = (float) unpackInt16(data, 6) / 100.f;
        break;
    }
    case UVHPU_PACK2: {
        uvhpu->msg2.vout = (float) unpackInt16(data, 0) / 100.f;
        uvhpu->msg2.tbat = (float) unpackInt16(data, 2) / 100.f;
        uvhpu->msg2.pbat = (float) unpackInt16(data, 4);
        uvhpu->msg2.status = data[6];
        break;
    }
    case UVHPU_PACK3: {
        memcpy(&uvhpu->msg3.cbat, data, 8);
        break;
    }
    case UVHPU_PACK4: {
        memcpy(&uvhpu->msg4.res_bar, data, 8);
        break;
    }
    case UVHPU_PACK5: {
        memcpy(&uvhpu->msg5.ibat_flt, data, 8);
        break;
    }
    case UVHPU_PACK6: {
        memcpy(&uvhpu->msg6.cbat_res, data, 8);
        break;
    }
    case UVHPU_PACK7: {
        uvhpu->msg7.life_cycles = unpackInt16(data, 0);
        memcpy(&uvhpu->msg7.cbat_mod, data + 2, 4);
        break;
    }
    }
}

void processMCELLPackage(const uint32_t &can_id, const uint8_t *data, const uint8_t &idx)
{
    auto *mcell = &_mcell[idx];

    switch (can_id) {
    case MCELL_PACK1: {
        mcell->vbat = (float) unpackInt16(data, 0) / 100.f;
        mcell->tbat = (float) unpackInt16(data, 2) / 100.f;
        mcell->tpcb = (float) unpackInt16(data, 4) / 100.f;
        mcell->status = data[6];
        break;
    }
    case MCELL_PACK2: {
        memcpy(mcell->cell, data, 8);
        break;
    }
    case MCELL_PACK3: {
        memcpy(mcell->cell + 4, data, 8);
        break;
    }
    }
}

EXPORT void canAuxHandler(const uint8_t *data, size_t size)
{
    if (size != PACK_CAN_SIZE) {
        return;
    }

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
    can_id &= 0x7FFFFFFF; // 32nd bit is ext/std flag

    uint8_t can_data[8] = {};
    for (uint8_t i = 0; i < 8; i++) {
        can_data[i] = data[4 + i]; // 4 is data position
    }

    const uint32_t now = time_ms();

    //CAN ID 0xFF
    if ((can_id & 0xFF) == VESC_ID) {
        t_vescResponse = now;
    }

    //CAN ID 0XFFFF
    switch (can_id & 0xFFFF) {
    case UVHPU_PACK1:
    case UVHPU_PACK2:
    case UVHPU_PACK3:
    case UVHPU_PACK4:
    case UVHPU_PACK5:
    case UVHPU_PACK6:
    case UVHPU_PACK7: {
        //printf("uvhpu %x", can_id);
        t_uvhpuResponse[0] = now;
        processUVHPUackage(can_id, can_data, 0);
        break;
    }
    case UVHPU_PACK1 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK2 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK3 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK4 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK5 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK6 + 1 * UVHPU_SHIFT:
    case UVHPU_PACK7 + 1 * UVHPU_SHIFT: {
        //printf("uvhpu %x", can_id);
        t_uvhpuResponse[1] = now;
        can_id -= 1 * UVHPU_SHIFT;
        processUVHPUackage(can_id, can_data, 1);
        break;
    }
    case MCELL_PACK1:
    case MCELL_PACK2:
    case MCELL_PACK3: {
        //printf("mcell %x", can_id);
        t_mcellResponse[0] = now;
        processMCELLPackage(can_id, can_data, 0);
        break;
    }
    case MCELL_PACK1 + 1 * MCELL_SHIFT:
    case MCELL_PACK2 + 1 * MCELL_SHIFT:
    case MCELL_PACK3 + 1 * MCELL_SHIFT: {
        //printf("mcell %x", can_id);
        t_mcellResponse[1] = now;
        can_id -= 1 * MCELL_SHIFT;
        processMCELLPackage(can_id, can_data, 1);
        break;
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
        printf("IFC-%s, wrong telemetry wing packet\n", txt_dev);
        return;
    }

    //TODO CRC
    if (data[0] == (MSG7_ID | ((NODE_WING_ID << 4) & 0xF0))) {
        memcpy(&_wing, &data[1], sizeof(WING_DATA));
    }
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

void print_mcell(const uint8_t &idx)
{
    printf("v_bat: %.2f", _mcell[idx].vbat);
    printf("t_bat: %.2f", _mcell[idx].tbat);
    printf("t_pcb: %.2f", _mcell[idx].tpcb);
    printf("state: %u", _mcell[idx].status);

    printf("C[1]: %.2f", _mcell[idx].cell_volt(0));
    printf("C[2]: %.2f", _mcell[idx].cell_volt(1));
    printf("C[3]: %.2f", _mcell[idx].cell_volt(2));
    printf("C[4]: %.2f", _mcell[idx].cell_volt(3));

    printf("C[5]: %.2f", _mcell[idx].cell_volt(4));
    printf("C[6]: %.2f", _mcell[idx].cell_volt(5));
}

EXPORT void mcell_1()
{
    printf("IFC-%s, mcell_1...\n", txt_dev);
    print_mcell(0);
}

EXPORT void mcell_2()
{
    printf("IFC-%s, mcell_2...\n", txt_dev);
    print_mcell(1);
}

EXPORT void mcell_3()
{
    printf("IFC-%s, mcell_3...\n", txt_dev);
    print_mcell(0);
}

EXPORT void mcell_4()
{
    printf("IFC-%s, mcell_4...\n", txt_dev);
    print_mcell(1);
}

void print_uvhpu(const uint8_t &idx)
{
    printf("vbat: %.2f", _uvhpu[idx].msg1.vbat);
    printf("ibat: %.2f", _uvhpu[idx].msg1.ibat);
    printf("imon: %.2f", _uvhpu[idx].msg1.imon);

    printf("vout: %.2f", _uvhpu[idx].msg2.vout);
    printf("tbat: %.2f", _uvhpu[idx].msg2.tbat);
    printf("pbat: %.2f", _uvhpu[idx].msg2.pbat);
    printf("status: %u", _uvhpu[idx].msg2.status);

    printf("cbat: %.2f", _uvhpu[idx].msg3.cbat);
    printf("ebat: %.2f", _uvhpu[idx].msg3.ebat);

    printf("res_bar: %.2f", _uvhpu[idx].msg4.res_bar);
    printf("v_res: %.2f", _uvhpu[idx].msg4.v_res);

    printf("ibat_filt: %.2f", _uvhpu[idx].msg5.ibat_flt);
    printf("vbat_filt: %.2f", _uvhpu[idx].msg5.vbat_flt);

    printf("cbat_res: %.2f", _uvhpu[idx].msg6.cbat_res);
    printf("ebat_res: %.2f", _uvhpu[idx].msg6.ebat_res);

    printf("life_cycles: %u", _uvhpu[idx].msg7.life_cycles);
    printf("cbat_mod: %.2f", _uvhpu[idx].msg7.cbat_mod);
}

EXPORT void uvhpu_1()
{
    printf("IFC-%s, uvhpu_1...\n", txt_dev);
    print_uvhpu(0);
}

EXPORT void uvhpu_2()
{
    printf("IFC-%s, uvhpu_2...\n", txt_dev);
    print_uvhpu(1);
}

EXPORT void uvhpu_3()
{
    printf("IFC-%s, uvhpu_3...\n", txt_dev);
    print_uvhpu(0);
}

EXPORT void uvhpu_4()
{
    printf("IFC-%s, uvhpu_4...\n", txt_dev);
    print_uvhpu(1);
}

EXPORT void wing_l()
{
    printf("IFC-%s, wing...\n", txt_dev);

    printf("v_temp1: %d", _wing.volz_temp[0]);
    printf("v_temp2: %d", _wing.volz_temp[1]);

    printf("v_pos1: %u", _wing.volz_pos[0]);
    printf("v_pos2: %u", _wing.volz_pos[1]);

    printf("nav_temp: %d", _wing.gyro_temp);
}

EXPORT void wing_r()
{
    printf("IFC-%s, wing...\n", txt_dev);

    printf("v_temp1: %d", _wing.volz_temp[0]);
    printf("v_temp2: %d", _wing.volz_temp[1]);

    printf("v_pos1: %u", _wing.volz_pos[0]);
    printf("v_pos2: %u", _wing.volz_pos[1]);

    printf("nav_temp: %d", _wing.gyro_temp);
}

EXPORT void pu_on()
{
    printf("IFC-%s, pu-on...\n", txt_dev);
    uint8_t msg[1] = {1};

    sendCmdToCan(UVHPU_CMD_PON, msg, 1);
    sendCmdToCan(UVHPU_CMD_PON + UVHPU_SHIFT, msg, 1);
}

EXPORT void pu_off()
{
    printf("IFC-%s, pu-ff...\n", txt_dev);
    uint8_t msg[1] = {0};

    sendCmdToCan(UVHPU_CMD_PON, msg, 1);
    sendCmdToCan(UVHPU_CMD_PON + UVHPU_SHIFT, msg, 1);
}
