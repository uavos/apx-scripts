#define TASK_DELAY_MS     20

#define PORT_ID_TLM_SYNC  42
#define PORT_ID_MHX       44

#define CMD_PWR_BAL       22


new txt_dev{1} = "L"

//----------------------------TelemetrySync-----------------------
const MSG_COUNT = 6;
new MSG_ID[MSG_COUNT] = [0, 3, 1, 4, 2, 5];

const SLAVE_NODE_COUNT = 2;
new SLAVE_NODE_ID[SLAVE_NODE_COUNT] = [3, 4];

new idx_msg = 0;
new idx_node = 0;

//----------------------------OnTask------------------------------
new schedule_sync_tlm_timer =   0;
new schedule_save_tlm_timer =   0;

const SCHEDULE_SYNC_TLM_TIMEOUT = 100;
const SCHEDULE_SAVE_TLM_TIMEOUT = 200;

new send_error =                0;
new size_error =                0;

new bool:g_SyncTelemetry =      true;

new Float: TLM_DIV =            1.0;

//----------------------------tlmIfc------------------------------
#define NODE_ID_IFC_L           3               //L=1
#define NODE_ID_IFC_R           4               //R=2

#define TLM_IFC_PACK_SIZE       8

new mc_pwr[2] = [0,]
new sp_pwr[2] = [0,]
new vc_pwr[2] = [0,]

const idx_mcell_pwr =           f_radar_dx;
const idx_mppt_pwr =            f_radar_dy;
const idx_vesc_pwr =            f_radar_dz;


//----------------------------link---------------------------------
const LINK_AVAILABLE_TIMEOUT = 10;              //seconds
new g_tpLastLinkAvailable;

bool:getLinkAvailable()
{
    new Float:statusModem = get_var(f_status_modem);
    new now = time();
    if(statusModem == 1.0)
        g_tpLastLinkAvailable = now;
    else if(now - g_tpLastLinkAvailable > LINK_AVAILABLE_TIMEOUT * 1000)
        return false;

    return true;
}

main()
{
    new now = time();

    schedule_sync_tlm_timer = now;
    schedule_save_tlm_timer = now;
    g_tpLastLinkAvailable = now;

    serial_listen(PORT_ID_MHX,  "@tlmIfcHandler");
}

@OnTask()
{
    new now = time();
    if (g_SyncTelemetry && now - schedule_sync_tlm_timer > SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV) {
        schedule_sync_tlm_timer = now;
        telemetrySync();
    }

    if (now - schedule_save_tlm_timer > SCHEDULE_SAVE_TLM_TIMEOUT) {
        schedule_save_tlm_timer = now;

        //save telemetry
        set_var(idx_mcell_pwr, (mc_pwr[0] + mc_pwr[1]), true);
        set_var(idx_mppt_pwr,  (sp_pwr[0] + sp_pwr[1]), true);
        set_var(idx_vesc_pwr,  (vc_pwr[0] + vc_pwr[1]), true);
    }

    new isLinkAvailable = getLinkAvailable();
    if (!isLinkAvailable && g_SyncTelemetry) {
        printf("MHX-%s: Datalink off\n", txt_dev);
        g_SyncTelemetry = false;
    }

    return TASK_DELAY_MS;
}

telemetrySync()
{
    new data{2};
    data{0} = SLAVE_NODE_ID[idx_node];
    data{1} = MSG_ID[idx_msg];

    if (idx_node++ >= 1) {
        idx_node = 0;
    }
    if (idx_msg++ >= 5) {
        idx_msg = 0;
    }

    new bool:result = serial_write(PORT_ID_TLM_SYNC, data, 2, serialmode:LAN);

    if (!result && send_error++ < 5) {
        printf("MHX-%s: send telemetry error...\n", txt_dev);
    }
}

unpackInt2(data{}, index)
{
    new val = data{index} + (data{index+1} << 8);
    if (val & 0x8000)
        val = (-1)*(0x10000 - val);
    return val;
}


processTlmIfcPackage(ifc_id, data{})
{
    mc_pwr[ifc_id] = unpackInt2(data, 1);
    sp_pwr[ifc_id] = unpackInt2(data, 3);
    vc_pwr[ifc_id] = unpackInt2(data, 5);
}

forward @tlmIfcHandler(cnt)
@tlmIfcHandler(cnt)
{
    if (cnt != TLM_IFC_PACK_SIZE && size_error++ < 5) {
        printf("MHX-%s, wrong telemetry wing packet\n", txt_dev);
        return;
    }

    new data{8};

    for (new i = 0; i < cnt; i++) {
        data{i} = serial_byte(i);
    }

    if (data{0} == (NODE_ID_IFC_L<<4 & 0xF0)) {
        processTlmIfcPackage(0, data);
    }

    if (data{0} == (NODE_ID_IFC_R<<4 & 0xF0)) {
        processTlmIfcPackage(1, data);
    }
}

forward @reset_error()
@reset_error()
{
    printf("MHX-%s: send(%d)\n", txt_dev, send_error);
    printf("MHX-%s: size(%d)\n", txt_dev, size_error);

    send_error = 0;
    size_error = 0;
}

forward @vm_status()
@vm_status()
{
    printf("MHX-%s:Ok...\n", txt_dev);
}

forward @tlm_f()
@tlm_f()
{
    TLM_DIV = 0.5;
    new TIMEOUT = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    g_SyncTelemetry = true;
    printf("MHX-%s: fast(%d)\n", txt_dev, TIMEOUT);
}

forward @tlm_n()
@tlm_n()
{
    TLM_DIV = 1.0;
    new TIMEOUT = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    g_SyncTelemetry = true;
    printf("MHX-%s: normal(%d)\n", txt_dev, TIMEOUT);
}

forward @tlm_l()
@tlm_l()
{
    TLM_DIV = 5.0;
    new TIMEOUT = SCHEDULE_SYNC_TLM_TIMEOUT * TLM_DIV;
    g_SyncTelemetry = true;
    printf("MHX-%s: low(%d)\n", txt_dev, TIMEOUT);
}

forward @tlm_off()
@tlm_off()
{
    g_SyncTelemetry = false;
    printf("MHX-%s: telemetry off\n", txt_dev);
}
