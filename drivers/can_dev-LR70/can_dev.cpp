/// CAN PROTOCOLS///

#include <apx.h>

using throttle = Mandala<mandala::cmd::nav::rc::thr>;

#define PORT_ID 5

#define CAN_PACK_SIZE 12 // vcp can, without dlc info

//--------------- VESC --------------------
#define VESC_TAIL_CAN_ID 0x75 // 0x24 // VESC ID 36
#define VESC_GEN_CAN_ID 0x25  // VESC ID 37
#define STATUS_MSG_1 0x09
#define STATUS_MSG_2 0x0E
#define STATUS_MSG_3 0x0F
#define STATUS_MSG_4 0x10
#define STATUS_MSG_5 0x1B
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM 3
//-------------------------------------------

static constexpr const port_id_t port_id{PORT_ID};

int main()
{
    throttle(); // subscribe
    receive(port_id, "on_serial");

    task("on_task", 100);

    return 0;
}

bool processGenPackage(uint32_t can_id, const uint8_t *data)
{
    if ((can_id & 0xFF) == VESC_GEN_CAN_ID) {
        uint16_t msg_id = (can_id >> 8) & 0xFF;

        switch (msg_id) {
            // printf("VESC_GEN_CAN_ID = %d, MSG_ID = %d", VESC_GEN_CAN_ID, msg_id);
        case STATUS_MSG_4: {
            float temp_fet = ((data[0] << 8) | data[1]) / 10.0;
            // printf("msg4, temp_fet %.1f", temp_fet);
            float temp_mot = ((data[2] << 8) | data[3]) / 10.0; // wrong readings???
            // printf("msg4, temp_motor %.1f", temp_mot);
            float curr_in = ((data[4] << 8) | data[5]) / 10.0;
            // printf("msg4, curr_in %.1f", curr_in);
            float pid_pos_now = ((data[6] << 8) | data[7]) / 50.0;
            // printf("msg4, pid_pos_now %.1f", pid_pos_now);
            break;
        }
        }
        return true;
    }
    return false;
}

bool processTailPackage(uint32_t can_id, const uint8_t *data)
{
    if ((can_id & 0xFF) == VESC_TAIL_CAN_ID) {
        uint16_t msg_id = (can_id >> 8) & 0xFF;

        switch (msg_id) {
        case STATUS_MSG_1: {
            uint32_t rpm = (uint32_t) ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
            printf("msg1, rpm %d", rpm);
            float current = ((data[4] << 8) | data[5]) / 10.0;
            // printf("msg1, current %.1f", current);
            float duty = ((data[6] << 8) | data[7]) / 1000.0;
            // printf("msg1, duty %.2f", duty);
            break;
        }
        case STATUS_MSG_2: {
            float apm_hours = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3])
                              / 10000.0;
            // printf("msg3, apm_hours %.2f", apm_hours);
            float apm_hours_charged = ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7])
                                      / 10000.0;
            // printf("msg3, apm_hours_charged %.2f", apm_hours_charged);
            break;
        }
        case STATUS_MSG_3: {
            float watt_hours = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3])
                               / 10000.0;
            // printf("msg3, watt_hours %.2f", watt_hours);
            float watt_hours_charged = ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7])
                                       / 10000.0;
            // printf("msg3, watt_hours_charged %.2f", watt_hours_charged);
            break;
        }
        case STATUS_MSG_4: {
            float temp_fet = ((data[0] << 8) | data[1]) / 10.0;
            // printf("msg4, temp_fet %.1f", temp_fet);
            float temp_mot = ((data[2] << 8) | data[3]) / 10.0; // wrong readings???
            // printf("msg4, temp_motor %.1f", temp_mot);
            float curr_in = ((data[4] << 8) | data[5]) / 10.0;
            // printf("msg4, curr_in %.1f", curr_in);
            float pid_pos_now = ((data[6] << 8) | data[7]) / 50.0;
            // printf("msg4, pid_pos_now %.1f", pid_pos_now);
            break;
        }
        case STATUS_MSG_5: {
            float voltage = ((data[4] << 8) | data[5]) / 10.0;
            uint32_t tacho = (uint32_t) ((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
                                         | data[3]);
            printf("msg5, voltage %.1f", voltage);
            // printf("msg5, tacho %X", tacho);

            break;
        }
        }
        return true;
    }

    return false;
}

EXPORT void on_serial(const uint8_t *data, size_t size)
{
    if (size != CAN_PACK_SIZE)
        return;

    uint32_t can_id = (uint32_t) (data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));

    uint8_t packData[8];
    for (uint8_t i = 0; i < 8; i++) {
        packData[i] = data[4 + i]; // 4 is data position
    }

    switch (can_id & 0xFF) {
    case VESC_TAIL_CAN_ID:
        processTailPackage(can_id, packData);
        break;

    case VESC_GEN_CAN_ID:
        processGenPackage(can_id, packData);
        break;

    default:
        break;
    }
}

// void addToPackedArray(uint8_t *data, uint8_t index, uint32_t value, uint8_t n)
//{
//     for (uint8_t i = 0; i < n; i++)
//     {
//         data[index + i] = (value >> (8 * i)) & 0xFF;
//     }
// }

void serializeInt(uint8_t *data, uint8_t index, uint32_t value)
{
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t shift = 8 * (4 - i - 1);
        data[index + i] = (value >> shift) & 0xFF;
    }
}

void setCurrent(float val)
{
    uint8_t msg[4 + 4];
    uint32_t current = val * 1000;

    msg[0] = VESC_TAIL_CAN_ID; //!!!DONT FORGET TO CHANGE TO CORRECT ONE!!!!!
    msg[1] = CAN_PACKET_SET_CURRENT;
    msg[2] = 0;
    msg[3] = 0x80; // IDE (bit 7) 1=ext,0=std;

    // addToPackedArray(msg, 4, val * 1000, 4);
    serializeInt(msg, 4, current);

    send(port_id, msg, 8, true); // true or false?
}

void setRPM(float val)
{
    uint8_t msg[4 + 4]; // ext id + DATA
    uint32_t current = val * 10000;

    msg[0] = VESC_TAIL_CAN_ID; //!!!DONT FORGET TO CHANGE TO CORRECT ONE!!!!!
    msg[1] = CAN_PACKET_SET_RPM;
    msg[2] = 0;
    msg[3] = 0x80; // IDE (bit 7) 1=ext,0=std

    serializeInt(msg, 4, current);
    // printf("%d", msg[5]);

    send(port_id, msg, 8, true); // true or false?
}

EXPORT void on_task()
{
    float thr = throttle::value();
    setRPM(thr);
    // setCurrent(thr);
}
