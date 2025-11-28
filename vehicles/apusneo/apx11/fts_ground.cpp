
#include <apx.h>

static constexpr const port_id_t LTE_port_id{43};
static constexpr const port_id_t FTS_port_id{44};

static constexpr const uint8_t FTS_DATA_SIZE{20};

uint32_t g_timePoint = 0;
int g_lte_step = 0;
bool g_lte_configured = false;

constexpr uint32_t LTE_DELAY_MS = 5000; // 5 сек

void send_cmd(const char *cmd)
{
    send(LTE_port_id, cmd, strlen(cmd), true);
    printf("SEND: %s", cmd);
}

void send_lte_message(const uint8_t *data, size_t len)
{
    char cmd[128];
    size_t cmd_len = 0;
    size_t i;

    // формируем начало команды с топиком жестко
    const char prefix[] = "AT+MPUB=\"UzuF9gZvg\",2,0,\"";
    for (i = 0; prefix[i] != '\0'; i++)
        cmd[cmd_len++] = prefix[i];

    // копируем данные FTS
    for (i = 0; i < len && cmd_len < sizeof(cmd) - 3; i++) {
        char c = data[i];
        if (c >= 32 && c <= 126) // печатные символы
            cmd[cmd_len++] = c;
        else
            cmd[cmd_len++] = '?';
    }

    // закрываем кавычки и добавляем \r\n
    cmd[cmd_len++] = '"';
    cmd[cmd_len++] = '\r';
    cmd[cmd_len++] = '\n';
    cmd[cmd_len] = '\0';

    send(LTE_port_id, cmd, cmd_len, true);
    printf("SEND: %s", cmd);
}

EXPORT void configure_LTE()
{
    uint32_t now = time_ms();

    // если ещё не прошло 5 секунд с последней команды или уже настроено, выходим
    if (now - g_timePoint < LTE_DELAY_MS || g_lte_configured)
        return;

    switch (g_lte_step) {
    case 0:
        send_cmd("AT+CPIN=\"4424\"\r\n");
        break;

    case 1:
        send_cmd("AT+MCONFIG=\"GroundModem\",\"f9aIrgWvR\",\"fraIrgZDgz\"\r\n");
        break;

    case 2:
        send_cmd("AT+MIPSTART=\"iot.dfrobot.com\",\"1883\"\r\n");
        break;

    case 3:
        send_cmd("AT+MCONNECT=1,60\r\n");
        break;

    case 4:
        send_cmd("AT+MSUB=\"UzuF9gZvg\",0\r\n");
        break;

    case 5:
        printf("LTE init complete!\n");
        g_lte_configured = true;
        return; // завершили настройку
    }

    // переход к следующему шагу
    g_lte_step++;
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
    if (size == 0)
        return;

    char buffer[128];
    size_t len = (size < sizeof(buffer) - 1) ? size : sizeof(buffer) - 1;

    // копируем данные в локальный буфер
    for (size_t i = 0; i < len; i++)
        buffer[i] = data[i];
    buffer[len] = '\0';

    // находим индекс последней запятой
    size_t last_comma_index = 0;
    int found_comma = 0;
    for (size_t i = 0; i < len; i++) {
        if (buffer[i] == ',') {
            last_comma_index = i;
            found_comma = 1;
        }
    }

    if (!found_comma) {
        printf("LTE: неверный формат сообщения\n");
        return;
    }

    // вычисляем начало payload
    size_t payload_start = last_comma_index + 1;
    size_t payload_len = len - payload_start;

    // убираем возможные \r\n в конце
    while (payload_len > 0
           && (buffer[payload_start + payload_len - 1] == '\n'
               || buffer[payload_start + payload_len - 1] == '\r')) {
        payload_len--;
    }

    // отправляем в FTS
    send(FTS_port_id, (uint8_t *) &buffer[payload_start], payload_len, true);

    // печать для отладки
    char debug_buffer[128];
    size_t debug_len = (payload_len < sizeof(debug_buffer) - 1) ? payload_len
                                                                : sizeof(debug_buffer) - 1;

    // копируем payload в debug_buffer и добавляем нуль-терминатор
    for (size_t i = 0; i < debug_len; i++)
        debug_buffer[i] = buffer[payload_start + i];
    debug_buffer[debug_len] = '\0';

    printf("SEND TO FTS: %s\n", debug_buffer);
}

EXPORT void on_rx_from_fts(const uint8_t *data, size_t size)
{
    if (size != FTS_DATA_SIZE) {
        printf("FTS data size error: %u", size);
        return;
    }

    // отправляем по LTE сразу после приёма
    if (!g_lte_configured) {
        printf("LTE not configured yet\n");
        return;
    }

    send_lte_message((uint8_t *) data, size);
}
