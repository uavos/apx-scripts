
#include <apx.h>

//username: f9aIrgWvR
//password: fraIrgZDgz
//topic to publish UzuF9gZvg
//topic to subscribe p0Kq0dZvg

static constexpr const port_id_t LTE_port_id{20};
static constexpr const port_id_t FTS_port_id{10};

static constexpr const uint8_t FTS_DATA_SIZE{16};

uint32_t g_timePoint = 0;
int g_lte_step = 0;
bool g_lte_configured = false;
bool g_connack_ok = false;
bool g_connect_ok = false;
bool g_suback_ok = false;

constexpr uint32_t LTE_DELAY_MS = 7000;

void send_cmd(const char *cmd)
{
    send(LTE_port_id, cmd, strlen(cmd), true);
    //printf("SEND: %s", cmd);
}

void send_lte_message(const uint8_t *data, size_t size)
{
    char cmd[200];
    size_t p = 0;
    const char hex[] = "0123456789ABCDEF";

    const char prefix[] = "AT+MPUB=\"UzuF9gZvg\",2,0,\"";
    for (size_t i = 0; prefix[i] != 0; i++)
        cmd[p++] = prefix[i];

    // Перевод каждого байта в два ASCII символа HEX
    for (size_t i = 0; i < size; i++) {
        uint8_t b = data[i];
        cmd[p++] = hex[b >> 4];
        cmd[p++] = hex[b & 0x0F];
    }

    // Закрыть кавычки и добавить \r\n
    cmd[p++] = '"';
    cmd[p++] = '\r';
    cmd[p++] = '\n';
    cmd[p] = 0;

    send(LTE_port_id, cmd, p, true);
    //printf("SEND: %s", cmd);
}

EXPORT void configure_LTE()
{
    uint32_t now = time_ms();

    // если ещё не прошло 5 секунд с последней команды или уже настроено, выходим
    if (now - g_timePoint < LTE_DELAY_MS || g_lte_configured)
        return;

    g_lte_step++;
    switch (g_lte_step) {
    case 1:
        send_cmd("AT\r\n");
        break; //should answer OK
    case 2:
        send_cmd("AT+MCONFIG=\"FlightModem\",\"f9aIrgWvR\",\"fraIrgZDgz\"\r\n");
        break; //should answer OK

    case 3:
        send_cmd("AT+MIPSTART=\"iot.dfrobot.com\",\"1883\"\r\n");
        break; //should answer OK, then CONNECT OK

    case 4:
        send_cmd("AT+MCONNECT=1,65535\r\n");
        break; //should answer OK, then CONNAK OK

    case 5:
        send_cmd("AT+MSUB=\"p0Kq0dZvg\",0\r\n");
        break; //should answer OK, then SUBACK

    case 6: {
        if (g_connack_ok && g_connect_ok && g_suback_ok) {
            printf("LTE init complete!\n");
            send_cmd("AT+MPUB=\"UzuF9gZvg\",2,0,\"UAV LTE connected\"\r\n");
            g_lte_configured = true;
            return;
        } else {
            printf("LTE init failed, retrying...\n");
            send_cmd("AT+RESET\r\n");
            g_connack_ok = false;
            g_connect_ok = false;
            g_suback_ok = false;
            g_lte_step = 0;
            break;
        }
    }
        return;
    }

    g_timePoint = now;
}

int main()
{
    receive(LTE_port_id, "on_rx_from_lte");
    receive(FTS_port_id, "on_rx_from_fts");

    schedule_periodic(task("configure_LTE"), 1000);

    return 0;
}

EXPORT void on_rx_from_lte(const uint8_t *data, size_t size)
{
    if (size == 14) {
        if (memcmp(&data[2], "CONNACK OK", 10) == 0) {
            printf("CONNACK OK received");
            g_connack_ok = true;
            return;
        }

        if (memcmp(&data[2], "CONNECT OK", 10) == 0) {
            printf("CONNECT OK received");
            g_connect_ok = true;
            return;
        }
    }

    if (size == 10) {
        if (memcmp(&data[2], "SUBACK", 6) == 0) {
            printf("SUBACK OK received");
            g_suback_ok = true;
            return;
        }
    }

    if (size < FTS_DATA_SIZE * 2) {
        //printf("LTE: message too short, size=%u", (unsigned) size);
        return;
    }

    if (memcmp(&data[2], "+MSUB:", 6) != 0) {
        //printf("LTE: unexpected beginning of message\n");
        return; // игнорируем чужие сообщения
    }

    char buffer[128];
    size_t len = (size < sizeof(buffer) - 1) ? size : sizeof(buffer) - 1;

    // копируем входные данные в локальный буфер
    for (size_t i = 0; i < len; i++)
        buffer[i] = data[i];
    buffer[len] = '\0';

    // находим последнюю запятую
    size_t last_comma_index = 0;
    int found_comma = 0;
    for (size_t i = 0; i < len; i++) {
        if (buffer[i] == ',') {
            last_comma_index = i;
            found_comma = 1;
        }
    }

    if (!found_comma) {
        //printf("LTE: неверный формат сообщения\n");
        return;
    }

    // начало полезной нагрузки
    size_t payload_start = last_comma_index + 1;
    size_t payload_len = len - payload_start;

    // убираем \r и \n в конце
    while (payload_len > 0
           && (buffer[payload_start + payload_len - 1] == '\n'
               || buffer[payload_start + payload_len - 1] == '\r')) {
        payload_len--;
    }

    // ожидаем, что payload — текстовый HEX (длина чётная)
    if (payload_len < 2 || (payload_len % 2) != 0) {
        //printf("LTE: HEX payload invalid length: %u\n", (unsigned) payload_len);
        return;
    }

    if (payload_len != 32)
        return;

    size_t byte_count = payload_len / 2;
    uint8_t bin[64];

    for (size_t i = 0; i < byte_count; i++) {
        char hi = buffer[payload_start + i * 2];
        char lo = buffer[payload_start + i * 2 + 1];

        int hi_val = (hi >= '0' && hi <= '9')   ? hi - '0'
                     : (hi >= 'A' && hi <= 'F') ? hi - 'A' + 10
                     : (hi >= 'a' && hi <= 'f') ? hi - 'a' + 10
                                                : -1;

        int lo_val = (lo >= '0' && lo <= '9')   ? lo - '0'
                     : (lo >= 'A' && lo <= 'F') ? lo - 'A' + 10
                     : (lo >= 'a' && lo <= 'f') ? lo - 'a' + 10
                                                : -1;

        if (hi_val < 0 || lo_val < 0) {
            printf("LTE: invalid hex at %u\n", (unsigned) i);
            return;
        }

        bin[i] = (hi_val << 4) | lo_val;
    }

    // отправляем бинарные данные в FTS
    send(FTS_port_id, bin, byte_count, true);

    // debug вывод (HEX)
    printf("SENT to FTS: %d bytes\n", (int) byte_count);
    printf("starts with %02X", bin[0]);
    printf("ends with %02X", bin[15]);
}

EXPORT void on_rx_from_fts(const uint8_t *data, size_t size)
{
    if (size != FTS_DATA_SIZE) {
        printf("FTS data size error: %u", size);
        return;
    }

    // отправляем по LTE сразу после приёма
    if (!g_lte_configured) {
        //printf("LTE not configured yet\n");
        return;
    }

    //send_lte_message((uint8_t *) data, size); //dont send ping all the time
}
