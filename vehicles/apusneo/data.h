#ifndef DATA_H
#define DATA_H

#include <cstdint>
#include <vector>
#include <map>
#include <string>


#define PORT_ID_GCU_W           40
#define PORT_ID_GCU_R           41
#define PORT_ID_TLM_SYNC        42
#define PORT_ID_WING            43


enum NodeId
{
    nNavC = 0,
    nNavL = 1,
    nNavR = 2,
    nIfcL = 3,
    nIfcR = 4,
    nMhxL = 5,
    nMhxR = 6,
};

enum MessageID
{
    mIfcL1 = 0,     //uvhpu-1, mcell-1, mppt-1
    mIfcL2 = 1,     //uvhpu-2, mcell-2, mppt-2
    mIfcL3 = 2,     //vesc-1,  volz-l
    mIfcR1 = 3,     //uvhpu-3, mcell-3, mppt-3
    mIfcR2 = 4,     //uvhpu-4, mcell-4, mppt-4
    mIfcR3 = 5,     //vesc-2,  volz-2
    mNavLR = 6,     //telemetry from wing
};
//------------------------------------------------------------------------
typedef struct {
    //pack 2
    int16_t vout;           //mult=0.01
    int16_t tbat;           //mult=0.01
    uint8_t status;
    //pack 3
    int32_t cbat;
    int32_t ebat;
    //pack 4
    uint16_t res_bat;
    uint16_t v_res;         //mult=0.01
    //pack 5
    int16_t ibat_flt;       //mult=0.01
    //pack 7
    int16_t h_pwr;          //mult=0.01
} __attribute__((packed)) t_uvhpu;
//------------------------------------------------------------------------
typedef struct {
    //pack 1
    int16_t vbat;           //mult=0.01
    int8_t tbat;
    int8_t tpcb;
    uint8_t status;
    //pack 2
    uint16_t c1;            //mult=0.001
    uint16_t c2;            //mult=0.001
    uint16_t c3;            //mult=0.001
    uint16_t c4;            //mult=0.001
    //pack 3
    uint16_t c5;            //mult=0.001
    uint16_t c6;            //mult=0.001
} __attribute__((packed)) t_mcell;
//------------------------------------------------------------------------
typedef struct {
    //pack 1
    int16_t vin;            //mult=0.01
    int16_t vout;           //mult=0.01
    int8_t temp;
    uint8_t status;
    //pack 2
    int16_t cin;            //mult=0.01
    int16_t cout;           //mult=0.01
} __attribute__((packed)) t_mppt;
//------------------------------------------------------------------------
typedef struct {
    int16_t crt;            //mult=0.01
    int8_t dc;
    int8_t ft;
    int8_t mt;
    int16_t crtin;          //mult=0.01
} __attribute__((packed)) t_vesc;
//------------------------------------------------------------------------
typedef struct {
    int8_t pos;
    int8_t temp;
} __attribute__((packed)) t_volz;
//------------------------------------------------------------------------
/*
 * Full packet structure:
 * | 1 byte            | N bytes   | 1 byte  |
 * | 0xF0    0x0F
 * | NodeId  MessageId | data      | crc8    |
 */
 //------------------------------------------------------------------------
typedef struct {
    uint8_t uvhpu_online        : 1;
    uint8_t mcell_online        : 1;
    uint8_t mppt_online         : 1;
    uint8_t unused              : 5;
    t_uvhpu uvhpu;
    t_mcell mcell;
    t_mppt mppt;
} __attribute__((packed)) t_ifc_lr_msg1;
//------------------------------------------------------------------------
typedef struct {
    uint8_t vesc_online         : 1;
    uint8_t volz_ail_online     : 1;
    uint8_t volz_elv_online     : 1;
    uint8_t volz_rud_online     : 1;
    uint8_t volz_pitch_online   : 1;
    uint8_t unused              : 3;
    t_vesc vesc;
    t_volz volz_ail;
    t_volz volz_sweep;
    t_volz volz_elv;
    t_volz volz_rud;
    int8_t wind_temp;
} __attribute__((packed)) t_ifc_lr_msg2;
//------------------------------------------------------------------------
enum CmdID
{
    PU_HEATER = 10,
    PU_PUMP
    SP_K
};
typedef struct {
    uint8_t uvhpu_heater;
    uint8_t uvhpu_pump;
    uint8_t mppt_k;
} __attribute__((packed)) t_ifc_lr_cmd;

static const uint8_t HEADER_SIZE = 1;
static const uint8_t CRC_SIZE = 1;

static const uint8_t PACKET_SIZE_UVHPU = sizeof(t_uvhpu);       //21
static const uint8_t PACKET_SIZE_MCELL = sizeof(t_mcell);       //17
static const uint8_t PACKET_SIZE_MPPT  = sizeof(t_mppt);        //10
static const uint8_t PACKET_SIZE_VESC  = sizeof(t_vesc);        //7
static const uint8_t PACKET_SIZE_VOLZ  = sizeof(t_volz);        //2

static const uint8_t PACKET_SIZE_MSG1 = sizeof(t_ifc_lr_msg1);  //49
static const uint8_t PACKET_SIZE_MSG2 = sizeof(t_ifc_lr_msg2);  //17

static const uint8_t PACKET_SIZE_MSG1_FULL = HEADER_SIZE + PACKET_SIZE_MSG1 + CRC_SIZE;
static const uint8_t PACKET_SIZE_MSG2_FULL = HEADER_SIZE + PACKET_SIZE_MSG2 + CRC_SIZE;

static const uint8_t M1_NL (mIfcL1 | (nIfcL << 4 & 0xF0));      //MESSAGE_ID AND NODE_ID
static const uint8_t M2_NL (mIfcL2 | (nIfcL << 4 & 0xF0));
static const uint8_t M3_NL (mIfcL3 | (nIfcL << 4 & 0xF0));
static const uint8_t M1_NR (mIfcR1 | (nIfcR << 4 & 0xF0));
static const uint8_t M2_NR (mIfcR2 | (nIfcR << 4 & 0xF0));
static const uint8_t M3_NR (mIfcR3 | (nIfcR << 4 & 0xF0));

static_assert(PACKET_SIZE_UVHPU == 21, "Check UVHPU struct...");
static_assert(PACKET_SIZE_MCELL == 17, "Check MCELL struct...");
static_assert(PACKET_SIZE_MPPT == 10,  "Check MPPT struct...");
static_assert(PACKET_SIZE_VESC == 7,  "Check VESC struct...");
static_assert(PACKET_SIZE_VOLZ == 2,  "Check VOLZ struct...");

static_assert(PACKET_SIZE_MSG1 == 49,  "Check MSG1 struct...");
static_assert(PACKET_SIZE_MSG2 == 17,  "Check MSG2 struct...");
//------------------------------------------------------------------------

#endif // DATA_H
