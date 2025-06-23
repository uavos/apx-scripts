#pragma once

#include <cstdint>
#include <string>

/*------------------------------------------------------------------------
 * Full packet structure:
 *
 * | 4 bytes        | N bytes   | 1 byte  |
 * | t_header (raw) | data      | crc8    |
 *
 * Detailed breakdown of t_header (32 bits):
 *
 * | 2 bytes          | 1 nibble | 1 nibble | 1 byte  |
 * | magic            | sender   | receiver | type    |
 *
 * Where:
 * - magic    : 16 bits — Magic constant to identify the packet
 * - sender   :  4 bits — Sender Node ID
 * - receiver :  4 bits — Receiver Node ID
 * - type     :  8 bits — Message Type ID
 *
 * The data section can vary in length (N bytes), depending on the message type.
 *------------------------------------------------------------------------*/

namespace haps {

constexpr const uint16_t MAGIC = 0x4D41; //'M''A'

constexpr const uint8_t PORT_ID_GCU_RW{40};
constexpr const uint8_t PORT_ID_TLM_SYNC{42};
constexpr const uint8_t PORT_ID_WING{43};

//------------------------------------------------------------------------
enum NodeID {
    nNavC = 0,
    nNavL = 1,
    nNavR = 2,
    nIfcL = 3,
    nIfcR = 4,
    nComL = 5,
    nComR = 6,
    nPYLD = 7,
    nGcu = 14,
    Broadcast = 15,
};

enum TypeID {
    msg_1 = 0,    //pu, mc, sp
    msg_2 = 1,    //pu, mc, sp
    msg_3 = 2,    //esc, srv
    msg_4 = 3,    //wing
    msg_5 = 4,    //esc, srv-l srv-r
    msg_6 = 5,    //pyld
    msg_cmd = 14, //from gcu_cmd
};

enum CmdGcu {
    pu1_h = 10,
    pu2_h,
    pu3_h,
    pu4_h,

    sp1_pwr,
    sp2_pwr,
    sp3_pwr,
    sp4_pwr,
};

struct t_header
{
    union {
        uint32_t raw;
        struct
        {
            uint16_t magic;       // Magic
            uint8_t sender : 4;   // NodeID
            uint8_t receiver : 4; // NodeID
            uint8_t type;         // TypeID
        } id;
    };

    constexpr t_header()
        : raw(0)
    {}

    constexpr t_header(uint16_t magic, uint8_t sender, uint8_t receiver, uint8_t type)
    {
        id.magic = magic;
        id.sender = sender & 0x0F;
        id.receiver = receiver & 0x0F;
        id.type = type;
    };

    bool is_broadcast() const { return id.receiver == 0xF; }

    bool operator==(const t_header &other) const { return this->raw == other.raw; }
} __attribute__((packed));

constexpr void h_pack_header(const t_header &header, uint8_t *buf)
{
    buf[0] = header.id.magic >> 8;
    buf[1] = header.id.magic & 0xFF;
    buf[2] = (header.id.sender << 4) | header.id.receiver;
    buf[3] = header.id.type;
}

constexpr t_header h_unpack_header(const uint8_t *buf)
{
    t_header header;
    header.id.magic = (buf[0] << 8) | buf[1];
    header.id.sender = (buf[2] >> 4) & 0x0F;
    header.id.receiver = buf[2] & 0x0F;
    header.id.type = buf[3];

    return header;
}
//------------------------------------------------------------------------
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
    uint16_t vres; //mult=0.01
    //pack 5
    int16_t ibat_flt; //mult=0.01
    //pack 7
    int16_t h_pwr; //mult=0.01
} __attribute__((packed)) t_pu;
//------------------------------------------------------------------------
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
} __attribute__((packed)) t_mc;
//------------------------------------------------------------------------
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
} __attribute__((packed)) t_sp;
//------------------------------------------------------------------------
typedef struct
{
    int16_t crt; //mult=0.01
    int8_t dc;
    int8_t ft;
    int8_t mt;
    int16_t crtin; //mult=0.01
} __attribute__((packed)) t_esc;
//------------------------------------------------------------------------
typedef struct
{
    int8_t pos;
    int8_t temp;
} __attribute__((packed)) t_srv;
//------------------------------------------------------------------------
typedef struct
{
    uint8_t pu_online : 1;
    uint8_t mc_online : 1;
    uint8_t sp_online : 1;
    uint8_t unused : 5;
    t_pu pu;
    t_mc mc;
    t_sp sp;
} __attribute__((packed)) t_msg1;
//------------------------------------------------------------------------
typedef struct
{
    uint8_t esc_online : 1;
    uint8_t srv_ail_online : 1;
    uint8_t srv_elv1_online : 1;
    uint8_t srv_elv2_online : 1;
    uint8_t srv_rud_online : 1;
    uint8_t srv_sweep_online : 1;
    uint8_t unused : 2;
    t_esc esc;
    t_srv srv_ail;
    t_srv srv_elv1;
    t_srv srv_elv2;
    t_srv srv_rud;
    t_srv srv_sweep;
    int8_t wing_temp;
    uint16_t nav_voltage; //mult=0.01
    uint16_t srv_voltage; //mult=0.01
} __attribute__((packed)) t_msg3;
//------------------------------------------------------------------------
typedef struct
{
    uint16_t voltage[2]; //sys, srv
    int8_t volz_temp[2];
    int16_t volz_pos[2];
    int8_t gyro_temp;
} __attribute__((packed)) t_msg4;
//------------------------------------------------------------------------
typedef struct
{
    uint8_t esc_online : 1;
    uint8_t srv_ail_online : 1;
    uint8_t srv_elv1_online : 1;
    uint8_t srv_elv2_online : 1;
    uint8_t srv_rud_online : 1;
    uint8_t srv_sweep_online : 1;
    uint8_t unused : 2;
    t_esc esc;
    t_srv srv_ail[2];
    t_srv srv_elv1[2];
    t_srv srv_elv2[2];
    t_srv srv_rud[2];
    t_srv srv_sweep[2];
    int8_t wing_temp[2];
    uint16_t nav_voltage[2]; //mult=0.01
    uint16_t srv_voltage[2]; //mult=0.01
} __attribute__((packed)) t_msg5;
//------------------------------------------------------------------------
typedef struct
{
    int8_t temp_room;
    int8_t temp_cam[2];
    int8_t temp_lens;
    int8_t temp_comp[2];
    int8_t temp_swmf1;
    t_srv srv_antenna;
    t_srv srv_valv;
} __attribute__((packed)) t_msg6;
//------------------------------------------------------------------------
constexpr uint8_t h_crc(const uint8_t *data, const uint8_t &size)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = crc & 0x80 ? (uint8_t) (crc << 1) ^ 0x31 : (uint8_t) (crc << 1);
    }
    return crc;
}
//------------------------------------------------------------------------
//IFC-L to GCU
constexpr const t_header IFCL_GCU_M1{MAGIC, NodeID::nIfcL, NodeID::nGcu, TypeID::msg_1}; //MSG1
constexpr const t_header IFCL_GCU_M2{MAGIC, NodeID::nIfcL, NodeID::nGcu, TypeID::msg_2}; //MSG2
constexpr const t_header IFCL_GCU_M3{MAGIC, NodeID::nIfcL, NodeID::nGcu, TypeID::msg_3}; //MSG3
constexpr const t_header IFCL_GCU_M5{MAGIC, NodeID::nIfcL, NodeID::nGcu, TypeID::msg_5}; //MSG5

//IFC-R to GCU
constexpr const t_header IFCR_GCU_M1{MAGIC, NodeID::nIfcR, NodeID::nGcu, TypeID::msg_1}; //MSG1
constexpr const t_header IFCR_GCU_M2{MAGIC, NodeID::nIfcR, NodeID::nGcu, TypeID::msg_2}; //MSG2
constexpr const t_header IFCR_GCU_M3{MAGIC, NodeID::nIfcR, NodeID::nGcu, TypeID::msg_3}; //MSG3
constexpr const t_header IFCR_GCU_M5{MAGIC, NodeID::nIfcR, NodeID::nGcu, TypeID::msg_5}; //MSG5

//NAV-L to IFC-L
constexpr const t_header NAVL_IFCL_M4{MAGIC, NodeID::nNavL, NodeID::nIfcL, TypeID::msg_4}; //MSG4

//NAV-R to IFC-R
constexpr const t_header NAVR_IFCR_M4{MAGIC, NodeID::nNavR, NodeID::nIfcR, TypeID::msg_4}; //MSG4

//PYLD to GCU
constexpr const t_header NPYLD_GCU_M6{MAGIC, NodeID::nPYLD, NodeID::nGcu, TypeID::msg_6}; // MSG6

//GCU-CMD
constexpr const t_header NGCU_CMD_BROADCAST{MAGIC, NodeID::nGcu, NodeID::Broadcast, TypeID::msg_cmd}; //CMD_BROADCAST
//------------------------------------------------------------------------
constexpr const uint8_t HEADER_SIZE = sizeof(t_header); //4
constexpr const uint8_t CRC_SIZE = 1;                   //1
constexpr const uint8_t MIN_CMD_SIZE = 6;               //6

constexpr const uint8_t PACKET_SIZE_PU = sizeof(t_pu);   //21
constexpr const uint8_t PACKET_SIZE_MC = sizeof(t_mc);   //17
constexpr const uint8_t PACKET_SIZE_SP = sizeof(t_sp);   //10
constexpr const uint8_t PACKET_SIZE_ESC = sizeof(t_esc); //7
constexpr const uint8_t PACKET_SIZE_SRV = sizeof(t_srv); //2

constexpr const uint8_t PACKET_SIZE_MSG1 = sizeof(t_msg1); //49
constexpr const uint8_t PACKET_SIZE_MSG3 = sizeof(t_msg3); //23
constexpr const uint8_t PACKET_SIZE_MSG5 = sizeof(t_msg5); //38

constexpr const uint8_t PACKET_SIZE_MSG1_FULL = HEADER_SIZE + PACKET_SIZE_MSG1 + CRC_SIZE; //54
constexpr const uint8_t PACKET_SIZE_MSG3_FULL = HEADER_SIZE + PACKET_SIZE_MSG3 + CRC_SIZE; //28
constexpr const uint8_t PACKET_SIZE_MSG5_FULL = HEADER_SIZE + PACKET_SIZE_MSG5 + CRC_SIZE; //43

static_assert(PACKET_SIZE_PU == 21, "Check PU struct...");
static_assert(PACKET_SIZE_MC == 17, "Check MC struct...");
static_assert(PACKET_SIZE_SP == 10, "Check SP struct...");
static_assert(PACKET_SIZE_ESC == 7, "Check ESC struct...");
static_assert(PACKET_SIZE_SRV == 2, "Check SRV struct...");

static_assert(PACKET_SIZE_MSG1 == 49, "Check MSG1 struct...");
static_assert(PACKET_SIZE_MSG3 == 23, "Check MSG3 struct...");
static_assert(PACKET_SIZE_MSG5 == 38, "Check MSG5 struct...");
//------------------------------------------------------------------------

} // namespace haps
