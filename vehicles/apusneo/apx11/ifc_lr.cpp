#include <apx.h>

#define IFC_LEFT TRUE //L-TRUE R-comment
//#define IFC_RIGHT              TRUE            //R-TRUE L-comment

#if defined IFC_LEFT
constexpr const char txt_dev{'L'};  //L
constexpr const uint8_t NODE_ID{3}; //L=1
constexpr const uint8_t MSG1_ID{0}; //L=0
constexpr const uint8_t MSG2_ID{1}; //L=1
constexpr const uint8_t MSG3_ID{2}; //L=2
//constexpr const uint8_t NODE_WING_ID{1};         //L=1
constexpr const port_id_t PORT_ID_CAN_AUX{20}; //L=20 //CAN   1Mb
#endif

#if defined IFC_RIGHT
constexpr const char txt_dev{'R'};  //R
constexpr const uint8_t NODE_ID{4}; //R=2
constexpr const uint8_t MSG1_ID{3}; //R=3
constexpr const uint8_t MSG2_ID{4}; //R=4
constexpr const uint8_t MSG3_ID{5}; //R=5
//constexpr const uint8_t NODE_WING_ID{2};         //R=2
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
//constexpr const uint8_t PACK_WING_SIZE{6};
//constexpr const uint8_t MSG7_ID{6}; //for nav-l and nav-r wing
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
//constexpr const uint16_t UVHPU_CMD_PON{UVHPU_ID + 12};
//constexpr const uint16_t UVHPU_CMD_RB{UVHPU_ID + 13};

//data
#pragma pack(1)
struct UVHPU
{
    struct
    {
        int16_t vbat;
        float ibat;
        int16_t imon;
    } MSG1;
    struct
    {
        int16_t vout;
        int16_t tbat;
        int16_t pbat;
        int16_t status;
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
        float ibat_flt;
        float vbat_flt;
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
        int16_t h_pwr;
    } MSG7;

    void *get_msg(const uint32_t &id)
    {
        switch (id) {
        case UVHPU_PACK1:
            return &MSG1;
        case UVHPU_PACK2:
            return &MSG2;
        case UVHPU_PACK3:
            return &MSG3;
        case UVHPU_PACK4:
            return &MSG4;
        case UVHPU_PACK5:
            return &MSG5;
        case UVHPU_PACK6:
            return &MSG6;
        case UVHPU_PACK7:
            return &MSG7;
        }

        return nullptr;
    };
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
constexpr const uint16_t MCELL_PACK4{MCELL_ID + 4};

#pragma pack(1)
struct MCELL
{
    int16_t vbat;
    int16_t tbat;
    int16_t tpcb;
    uint8_t status;
    uint16_t cell[12] = {};
    float cell_volt(uint8_t cell_idx) { return cell[cell_idx] / 1000.f; };

    void *get_msg(const uint32_t &id)
    {
        switch (id) {
        case MCELL_PACK1:
            return &vbat;
        case MCELL_PACK2:
            return cell;
        case MCELL_PACK3:
            return cell + 4;
        case MCELL_PACK4:
            return cell + 8;
        }

        return nullptr;
    };
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

bool responseState[11] = {}; //uvhpu1, mcell1, mppt1, uvhpu2, mcell2, mppt2, vesc, volz[4]

uint8_t size_error_gcu = 0;
uint8_t size_error_sync = 0;
uint8_t size_error_wing = 0;
//---------------------------------------------------------

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

    task("reset_error"); //GCS with terminal command `vmexec("reset_error")`

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

void packMsg1(const uint8_t &idx)
{
    t_ifc_lr_msg1 msg1 = {};

    msg1.uvhpu_online = responseState[idx * 3 + 0];
    msg1.mcell_online = responseState[idx * 3 + 1];
    msg1.mppt_online = responseState[idx * 3 + 2];

    //uvhpu
    //pack 2
    msg1.uvhpu.vout = _uvhpu[idx].MSG2.vout;
    msg1.uvhpu.tbat = _uvhpu[idx].MSG2.tbat;
    msg1.uvhpu.status = (uint8_t) _uvhpu[idx].MSG2.status;
    //pack 3
    msg1.uvhpu.cbat = (int32_t) _uvhpu[idx].MSG3.cbat;
    msg1.uvhpu.ebat = (int32_t) (_uvhpu[idx].MSG3.ebat * 100.f);
    //pack 4
    msg1.uvhpu.res_bat = (uint16_t) (_uvhpu[idx].MSG4.res_bar * 100.f);
    msg1.uvhpu.v_res = (uint16_t) (_uvhpu[idx].MSG4.v_res * 100.f);
    //pack 5
    msg1.uvhpu.ibat_flt = (int16_t) (_uvhpu[idx].MSG5.ibat_flt * 100.f);
    //pack 7
    msg1.uvhpu.h_pwr = _uvhpu[idx].MSG7.h_pwr;

    //uvhpu
    //pack 1
    msg1.mcell.vbat = int16_t(_mcell[idx].vbat);
    msg1.mcell.tbat = int8_t(_mcell[idx].tbat / 100.f);
    msg1.mcell.tpcb = int8_t(_mcell[idx].tpcb / 100.f);
    msg1.mcell.status = _mcell[idx].status;
    //pack 2
    msg1.mcell.c1 = _mcell[idx].cell[0];
    msg1.mcell.c2 = _mcell[idx].cell[1];
    msg1.mcell.c3 = _mcell[idx].cell[2];
    msg1.mcell.c4 = _mcell[idx].cell[3];
    //pack 3
    msg1.mcell.c5 = _mcell[idx].cell[4];
    msg1.mcell.c6 = _mcell[idx].cell[5];

    memcpy(&telemetryPack[1], &msg1, sizeof(t_ifc_lr_msg1));
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

    void *msg = nullptr;

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
        msg = _uvhpu[0].get_msg(can_id);
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
        msg = _uvhpu[1].get_msg(can_id);
        break;
    }
    case MCELL_PACK1:
    case MCELL_PACK2:
    case MCELL_PACK3:
    case MCELL_PACK4: {
        //printf("mcell %x", can_id);
        t_mcellResponse[0] = now;
        msg = _mcell[0].get_msg(can_id);
        break;
    }
    case MCELL_PACK1 + 1 * MCELL_SHIFT:
    case MCELL_PACK2 + 1 * MCELL_SHIFT:
    case MCELL_PACK3 + 1 * MCELL_SHIFT:
    case MCELL_PACK4 + 1 * MCELL_SHIFT: {
        //printf("mcell %x", can_id);
        t_mcellResponse[1] = now;
        msg = _mcell[1].get_msg(can_id);
        break;
    }
    }

    if (msg) {
        memcpy(msg, data, 8);
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
    (void) data;
    (void) size;
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
