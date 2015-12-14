/*
 * LightTelemetry implementation by KipK 
 *
 * Minimal one way telemetry protocol for really low bitrates (1200/2400 bauds). 
 * Effective for ground OSD, groundstation HUD and Antenna tracker (ie https://code.google.com/p/ghettostation/ )
 *             
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2hz @ 1200 bauds , 2hz >= 2400 bauds): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0   
 *     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0   
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2hz @ 1200bauds, 2hz >= 2400bauds): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0     
 *      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 */
#include "board.h"
#include "mw.h"

#define CYCLETIME             100
#define LTM_GFRAME_SIZE 18
#define LTM_AFRAME_SIZE 10
#define LTM_SFRAME_SIZE 11


static void send_LTM_Packet(uint8_t *LTPacket, uint8_t LTPacket_size)
{
    //calculate Checksum
    uint8_t LTCrc = 0x00;
    int i;
    for (i = 3; i < LTPacket_size-1; i++) {
        LTCrc ^= LTPacket[i];
    }
    LTPacket[LTPacket_size-1]=LTCrc;
    for (i = 0; i<LTPacket_size; i++) {
        serialWrite(core.lighttelemport,LTPacket[i]);
    }
    
}
// GPS frame
void send_LTM_Gframe()
{
    uint8_t ltm_gpsfix = f.GPS_FIX;
    if (ltm_gpsfix == 1)
        ltm_gpsfix = 3; // 3D fix
    uint8_t LTBuff[LTM_GFRAME_SIZE];
    //protocol: START(2 bytes)FRAMEID(1byte)LAT(cm,4 bytes)LON(cm,4bytes)SPEED(m/s,1bytes)ALT(cm,4bytes)SATS(6bits)FIX(2bits)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x47; // G ( gps frame at 5hz )
    //PAYLOAD
    LTBuff[3]=(GPS_coord[LAT] >> 8*0) & 0xFF;
    LTBuff[4]=(GPS_coord[LAT] >> 8*1) & 0xFF;
    LTBuff[5]=(GPS_coord[LAT] >> 8*2) & 0xFF;
    LTBuff[6]=(GPS_coord[LAT] >> 8*3) & 0xFF;
    LTBuff[7]=(GPS_coord[LON] >> 8*0) & 0xFF;
    LTBuff[8]=(GPS_coord[LON] >> 8*1) & 0xFF;
    LTBuff[9]=(GPS_coord[LON] >> 8*2) & 0xFF;
    LTBuff[10]=(GPS_coord[LON] >> 8*3) & 0xFF;
    LTBuff[11]=((uint8_t)round(GPS_speed/100) >> 8*0) & 0xFF;
    LTBuff[12]=(BaroAlt >> 8*0) & 0xFF;
    LTBuff[13]=(BaroAlt >> 8*1) & 0xFF;
    LTBuff[14]=(BaroAlt >> 8*2) & 0xFF;
    LTBuff[15]=(BaroAlt >> 8*3) & 0xFF;
    LTBuff[16]= ((GPS_numSat << 2)& 0xFF ) | (f.GPS_FIX & 0b00000011) ; // last 6 bits: sats number, first 2:fix type (0,1,2,3)
    send_LTM_Packet(LTBuff,LTM_GFRAME_SIZE);
}

//Sensors frame
static void send_LTM_Sframe() 
{
    uint8_t lt_flightmode;
    uint8_t lt_failsafe;
    uint8_t LTBuff[LTM_SFRAME_SIZE];
    
    // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
    // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 
    // 12: Circle, 13: RTH, 14: FollowMe, 15: LAND, 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown    
    if (f.PASSTHRU_MODE) lt_flightmode = 0;
    else if (f.GPS_HOME_MODE) lt_flightmode = 13;
    else if (f.GPS_HOLD_MODE) lt_flightmode = 10;
    else if (f.HEADFREE_MODE) lt_flightmode = 4;
    else if (f.BARO_MODE) lt_flightmode = 8;
    else if (f.ANGLE_MODE)lt_flightmode = 2;
    else if (f.HORIZON_MODE) lt_flightmode = 3;
    else lt_flightmode = 1; // Rate mode
    
    if (failsafeCnt>2) lt_failsafe = 1;
    else lt_failsafe = 0;
    //pack A frame    
    //A Frame: $T(2 bytes)A(1byte)PITCH(2 bytes)ROLL(2bytes)HEADING(2bytes)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x53; //S 
    //PAYLOAD
    LTBuff[3]=(vbat*100 >> 8*0) & 0xFF;                                                                    //vbat converted in mv
    LTBuff[4]=(vbat*100 >> 8*1) & 0xFF;
    LTBuff[5]= 0;                                                                           //consumed current. Not implemented in baseflight yet.
    LTBuff[6]= 0;
    LTBuff[7]=((uint8_t) (rssi*254)/1023 >> 8*0) & 0xFF;                                                   // rouding RSSI to 1 byte resolution.
    LTBuff[8]= 0;                                                                          // no airspeed in multiwii/baseflight
    LTBuff[9]= ((lt_flightmode << 2)& 0xFF ) | ((lt_failsafe << 1)& 0b00000010 ) | (f.ARMED & 0b00000001) ; // last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status.
    send_LTM_Packet(LTBuff,LTM_SFRAME_SIZE);
}

// Attitude frame
static void send_LTM_Aframe() 
{
    uint8_t LTBuff[LTM_AFRAME_SIZE];
    //A Frame: $T(2 bytes)A(1byte)PITCH(2 bytes)ROLL(2bytes)HEADING(2bytes)CRC(xor,1byte)
    //START
    LTBuff[0]=0x24; //$
    LTBuff[1]=0x54; //T
    //FRAMEID
    LTBuff[2]=0x41; //A 
    //PAYLOAD
    LTBuff[3]=((angle[1]/10) >> 8*0) & 0xFF;
    LTBuff[4]=((angle[1]/10) >> 8*1) & 0xFF;
    LTBuff[5]=((angle[0]/10) >> 8*0) & 0xFF;
    LTBuff[6]=((angle[0]/10) >> 8*1) & 0xFF;
    LTBuff[7]=(heading >> 8*0) & 0xFF;
    LTBuff[8]=(heading >> 8*1) & 0xFF;
    send_LTM_Packet(LTBuff,LTM_AFRAME_SIZE);
}

static bool lighttelemetryEnabled = false;

void initLightTelemetry(void)
{

 //to do: set hardwareserial or softserial output
    if (!feature(FEATURE_SOFTSERIAL))
        mcfg.lighttelemetry_port = TELEMETRY_PORT_UART;
        
    if (mcfg.lighttelemetry_port == TELEMETRY_PORT_SOFTSERIAL_1)
        core.lighttelemport = &(softSerialPorts[0].port);
    else if (mcfg.telemetry_port == TELEMETRY_PORT_SOFTSERIAL_2)
        core.lighttelemport = &(softSerialPorts[1].port);
    else
        core.lighttelemport = core.mainport;

}

static uint32_t ltm_lastCycleTime = 0;
static uint8_t ltm_cycleNum = 0;

void updateLightTelemetryState(void)
{
    bool State;
    
    if (mcfg.lighttelemetry_port == TELEMETRY_PORT_UART) 
    {
        if (!mcfg.telemetry_switch)
            State = f.ARMED;
        else
            State = rcOptions[BOXTELEMETRY];

        if (State != lighttelemetryEnabled) 
        {
            if (State)
                serialInit(mcfg.lighttelemetry_baudrate);
            else
                serialInit(mcfg.serial_baudrate);
            lighttelemetryEnabled = State;
        }
    }
}

void sendLightTelemetry(void)
{
    static uint8_t ltm_scheduler = 1;
    static uint8_t ltm_slowrate = 0;
    if ((!mcfg.telemetry_switch && !f.ARMED) || (mcfg.telemetry_switch && !rcOptions[BOXTELEMETRY]))
        return;
    if (serialTotalBytesWaiting(core.mainport) != 0)
        return;
    if (millis() - ltm_lastCycleTime >= CYCLETIME) {
        ltm_lastCycleTime = millis();
        ltm_cycleNum++;      
        if (mcfg.lighttelemetry_baudrate<2400) ltm_slowrate = 1;
        else ltm_slowrate=0;
        if (ltm_scheduler & 1) {    // is odd
            send_LTM_Aframe();
           // if (ltm_slowrate==0) send_LTM_Sframe(); 
        }
        else                        // is even
        {
                if (ltm_slowrate == 0 ) {
                send_LTM_Aframe();
                }
                if (ltm_scheduler % 4 == 0) send_LTM_Sframe();
                else send_LTM_Gframe();
        }
        ltm_scheduler++;
        if (ltm_scheduler > 10)
        ltm_scheduler = 1;
    }
}

