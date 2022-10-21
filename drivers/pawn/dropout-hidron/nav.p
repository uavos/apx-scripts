const LINK_AVAILABLE_TIMEOUT = 250;     //seconds
const LANDING_SET_INTERVAL = 5;         //seconds

//distance check
const Float:DC_RUNWAY_ALTITUDE = 0.0;   //meters
const Float:DC_MIN_DISTANCE = 600.0;    //meters
const Float:DC_MAX_ALTITUDE = 120.0;    //meters
const DC_TIMEOUT = 10;                  //seconds

//battery check
const Float:BC_MIN_VOLTAGE = 8.0;       //volts
const BC_TIMEOUT = 30;                  //seconds

//link check
const LC_TIMEOUT = 6 * 60;              //minutes

//release check
const Float:RELEASE_ALTITUDE = 30000.0; //meters
const Float:RELEASE_VDOWN = 20.0;       //m/s
const RC_TIMEOUT = 5;                   //seconds
const RELEASE_VAR = f_sw_sw4;           //idx

//burn check
const Float:BURN_ALTITUDE = 30050.0;    //meters
const BRC_TIMEOUT = 5;                  //seconds
const BURNING_TIME = 8;                 //seconds
const BURN_VAR = f_sw_sw2;              //idx

//
new g_tpLastLinkAvailable;
new g_tpLastDistanceCheck;
new g_tpLastBatteryCheck;
new g_tpLastLinkCheck;
new g_tpLastLanding;
new g_tpLastReleaseCheck;
new g_tpLastBurnCheck;
new g_tpBurnStarted;

new bool:g_lockout;
new bool:g_releaseLockout;
new bool:g_burnLockout;

main()
{
    new now = time();
    g_tpLastLinkAvailable = now;
    g_tpLastDistanceCheck = now;
    g_tpLastBatteryCheck = now;
    g_tpLastLinkCheck = now;
    g_tpLastLanding = now;
    g_tpLastReleaseCheck = now;
    g_tpLastBurnCheck = now;
    g_lockout = false;
    g_releaseLockout = false;
    g_burnLockout = false;
}

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

bool:distanceCheck()
{
    new Float:altitude = get_var(f_gps_hmsl) - DC_RUNWAY_ALTITUDE;
    new Float:dHome = get_var(f_dHome);
    new Float:dWpt = get_var(f_dWPT);
    new bool:tooFar = (dHome > DC_MIN_DISTANCE || dWpt > DC_MIN_DISTANCE);
    new bool:isLinkAvailable = getLinkAvailable();
    new now = time();
    if(!isLinkAvailable && altitude < DC_MAX_ALTITUDE && tooFar)
    {
        new elapsed = now - g_tpLastDistanceCheck;
        if(elapsed > DC_TIMEOUT * 1000)
            return false;
    }
    else
        g_tpLastDistanceCheck = now;

    return true;
}

bool:batteryCheck()
{
    new Float:voltage = get_var(f_Ve);
    new bool:isLinkAvailable = getLinkAvailable();
    new now = time();
    if(!isLinkAvailable && voltage < BC_MIN_VOLTAGE)
    {
        new elapsed = now - g_tpLastBatteryCheck;
        if(elapsed > BC_TIMEOUT * 1000)
            return false;
    }
    else
        g_tpLastBatteryCheck = now;

    return true;
}

bool:linkCheck()
{
    new bool:isLinkAvailable = getLinkAvailable();
    new now = time();
    if(!isLinkAvailable)
    {
        new elapsed = now - g_tpLastLinkCheck;
        if(elapsed > LC_TIMEOUT * 60 * 1000)
            return false;
    }
    else
        g_tpLastLinkCheck = now;

    return true;
}

bool:releaseCheck()
{
    new now = time();
    new Float:altitude = get_var(f_gps_hmsl);
    new Float:gps_vdown = get_var(f_gps_Vdown);
    new Float:notReleased = get_var(RELEASE_VAR);
    if(notReleased == 1.0 && (altitude > RELEASE_ALTITUDE || gps_vdown > RELEASE_VDOWN))
    {
        new elapsed = now - g_tpLastReleaseCheck;
        if(elapsed > RC_TIMEOUT * 1000)
            return false;
    }
    else
        g_tpLastReleaseCheck = now;

    return true;
}

bool:burnCheck()
{
    new now = time();
    new Float:altitude = get_var(f_gps_hmsl);
    if(altitude > BURN_ALTITUDE)
    {
        new elapsed = now - g_tpLastBurnCheck;
        if(elapsed > BRC_TIMEOUT * 1000)
            return false;
    }
    else
        g_tpLastBurnCheck = now;

    return true;
}

@OnTask()
{
    new now = time();

    new bool:isLinkAvailable = getLinkAvailable();

    if(isLinkAvailable)
        g_lockout = false;

    if(!isLinkAvailable && (now - g_tpLastLanding > LANDING_SET_INTERVAL * 1000))
    {
        print("Link unavailable, set mode to landing\n");
        set_var(f_mode, 8, true);   //landing
        g_tpLastLanding = now;
    }

    new bool:dc = distanceCheck();
    new bool:bc = batteryCheck();
    new bool:lc = linkCheck();

    if(!g_lockout && (!dc || !bc || !lc))
    {
        print("Safety check triggered, ERS on\n");
        set_var(f_ctrb_ers, 1.0, true);
        g_lockout = true;
    }

    new bool:rc = releaseCheck();
    if(!g_releaseLockout && !rc)
    {
        set_var(RELEASE_VAR, 0.0, true);
        g_releaseLockout = true;
        print("Baloon released\n");
    }

    new bool:brc = burnCheck();
    if(!g_burnLockout && get_var(BURN_VAR) == 0.0 && !brc)
    {
        g_tpBurnStarted = now;
        set_var(BURN_VAR, 1.0, true);
        print("Burning started\n");
    }

    if(!g_burnLockout && get_var(BURN_VAR) == 1.0 && now - g_tpBurnStarted > BURNING_TIME * 1000)
    {
        set_var(BURN_VAR, 0.0, true);
        g_burnLockout = true;
        print("Burning stopped\n");
    }

    return 1;
}
