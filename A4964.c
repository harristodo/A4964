//==============================================//
//			INCLUDE HEAER FILE					//
//==============================================//
#include "system.h"
/// FOR TEST
//----------------------------------------------
#define oCSL                STRn_SetLow()
#define oCSH                STRn_SetHigh()
#define SPI_Open()          SPI1_Open(SPI1_DEFAULT)
#define SPI_Close()         SPI1_Close()
#define SPI_ExchangeByte(x) SPI1_ExchangeByte(x)

//==============================================//
//				�Ȧs���w�q						//
//==============================================//
U16 regConfig[32] = {0};
U16 wrData = 0;
U16 receiveData = 0;


//==============================================//
//				�禡�쫬�ŧi					//
//==============================================//
U8 configStopOnFail (U8 esf);
U8 configVLRvoltage (U8 vlr);
U8 configGateVoltage (U8 vrg);
U8 configOperatingMode (U8 opm);
U8 configWakeMode (U8 lwk);
U8 configPWMSense (U8 ipi);
U8 configCurrentLimitEnable (U8 dil);
U8 configMotorMode (U8 mod);
U8 configLIN (U8 len);
U8 configSLP (U8 slp);
U8 configOvermodulation (U8 ovm);
U8 configDriveMode (U8 drm);
U8 configBRAKE (U8 brk);
U8 configDIR (U8 dir);
U8 configRUN (U8 run);
U8 configSpeedAccel (U16 accHzX10,U8 gain);
U8 configSpeedFlags (U8 dv,U8 df);
U8 configSpeed (U8 res, U16 lthr, U16 hthr);
U8 configSenseMaxThresh (U8 mit);
U8 configVDSOvervoltage(U16 mv);
U8 configVDSFaultQualifier(U8 mod,U16 ns);
U8 configPWM(U8 mod,U8 pmd,U16 freq);
U8 configDither(U8 ditherTime,U8 dwellTime,U8 stepCnt);
U8 configBridge(U8 gain, U16 dt);
U8 configGateCurrent(U8 gate, U8 cur1, U8 cur2);
U8 configGateSlew(U16 nsOn, U16 nsOff);
U8 configCurrentLimit (U16 ns,U8 scale);
U8 configStartupAlign(U16 ms,U8 dutyp);
U8 configStartupMotor(U8 coast,U8 rsc,U16 km,U8 rampt);
U8 configWindmill(U8 mod,U8 freq,U8 dutyp);
U8 configRamp(U16 startFreqX10,U8 startDuty,U16 endFreqX10, U8 endDuty);
U8 configRampStep(U16 stepms, U8 stepfreq);
U8 configBEMFDetectWindow(U16 degX10);
U8 configBEMF(U8 samplecnt, U8 filtertime);
U8 configCommSteadyPow2(S8 np,S8 ni);
U8 configCommTransientPow2(S8 np,S8 ni);
U8 configPhaseAdvance(U8 mod, U8 gain,U16 degX10);
U8 configDIAG (U8 DGS);
U8 configDataOutput (U8 RBS);

U8 A4964_SPI (U16 Data ,U8 WR ,U16 *rcvData);


const uint16_t A4964_CONFIG_EXL[] =
{

0x004C	,	//	CONFIG[0]
0x0800	,	//	CONFIG[1]
0x1028	,	//	CONFIG[2]
0x1800	,	//	CONFIG[3]
0x2000	,	//	CONFIG[4]
0x2800	,	//	CONFIG[5]
0x30FE	,	//	CONFIG[6]
0x387E	,	//	CONFIG[7]
0x407E	,	//	CONFIG[8]
0x4800	,	//	CONFIG[9]
0x5000	,	//	CONFIG[10]
0x58C8	,	//	CONFIG[11]
0x60C8	,	//	CONFIG[12] 0x60C8
0x6806	,	//	CONFIG[13]
0x7042	,	//	CONFIG[14]
0x7A84	,	//	CONFIG[15] 0x7A86
0x8038	,	//	CONFIG[16] 0x803A
0x894E	,	//	CONFIG[17]
0x9024	,	//	CONFIG[18]
0x9864	,	//	CONFIG[19] 0x9864
0xA028	,	//	CONFIG[20]  // A028
0xA83E	,	//	CONFIG[21] 0xA81E
0xB002	,	//	CONFIG[22]
0xB80E	,	//	CONFIG[23]
0xC000	,	//	CONFIG[24]
0xCA88	,	//	CONFIG[25]
0xD200	,	//	CONFIG[26]
0xD810	,	//	CONFIG[27]
0xE200	,	//	CONFIG[28]
0xE880	,	//	CONFIG[29]
0xF000	,	//	CONFIG[30]

0xF800,    // CONFIG[31]
};




//
//const uint16_t A4964_CONFIG_EXL[] =
//{
//
//0x004C	,	//	CONFIG[0]
//0x0800	,	//	CONFIG[1]
//0x1028	,	//	CONFIG[2]
//0x1800	,	//	CONFIG[3]
//0x2000	,	//	CONFIG[4]
//0x2800	,	//	CONFIG[5]
//0x30FE	,	//	CONFIG[6]
//0x387E	,	//	CONFIG[7]
//0x407E	,	//	CONFIG[8]
//0x4800	,	//	CONFIG[9]
//0x5000	,	//	CONFIG[10]
//0x58C8	,	//	CONFIG[11]
//0x60C8	,	//	CONFIG[12]
//0x6806	,	//	CONFIG[13]
//0x7042	,	//	CONFIG[14]
//0x7A86	,	//	CONFIG[15]
//0x803A	,	//	CONFIG[16]
//0x894E	,	//	CONFIG[17]
//0x9080	,	//	CONFIG[18]
//0x9864	,	//	CONFIG[19]
//0xA028	,	//	CONFIG[20]
//0xA81E	,	//	CONFIG[21]
//0xB002	,	//	CONFIG[22]
//0xB80E	,	//	CONFIG[23]
//0xC000	,	//	CONFIG[24]
//0xCA80	,	//	CONFIG[25]
//0xD200	,	//	CONFIG[26]
//0xD810	,	//	CONFIG[27]
//0xE200	,	//	CONFIG[28]
//0xE980	,	//	CONFIG[29]
//0xF000	,	//	CONFIG[30]
//
//0xF800,    // CONFIG[31]
//};



//==============================================//
// A4964_init(void)
//==============================================//
void A4964_init (void)
{
    U8 i,j,stat;
    U16 rData;
    
    for (i=0;i<32;i++)  // reg0 ~ reg31
        regConfig[i] = A4964_CONFIG_EXL[i];
    
    for (i=0;i<31;i++)  // reg0 ~ reg30
    {   // regConfig[0]=0x1234 - wData=1234 - stat=1
        printf("regConfig[%d]=%04X - ",i,regConfig[i]);
        stat = A4964_SPI(regConfig[i], cmdWR , &receiveData);
        printf("wrData=%04X - stat=%d \n\r",wrData,stat);
    }
}


//==============================================
// A4964_SPI		
//==============================================
U8 A4964_SPI (U16 Data ,U8 WR ,U16 *rcvData) 
{
    uint8_t sDataH, sDataL, cnt;
    uint16_t value,rDataH,rDataL;
    
    //------------------------------
    // set write bit
    if (WR)
        Data |= (1<<10);
    
    //------------------------------
    // calculate odd parity bit
    value = Data;
    Parity = false;   
    
    while(value)
    {
        Parity = !Parity;
        value = value & (value - 1);
    }
    
    if(!Parity)
        Data = Data | 0x0001;
    
    wrData = Data;
    //-----------------------------

    sDataH = Data >> 8;
    sDataL = Data;
    
    cnt = 3;
    
    SPI_Open();
    do
    {
        oCSL;
        rDataH = SPI_ExchangeByte(sDataH);
        rDataL = SPI_ExchangeByte(sDataL);
        oCSH;
        
        *rcvData = (rDataH<<8) | rDataL;
        
        if ((*rcvData & fbSE)==0)
            break;
        
    }while (cnt--);
    SPI_Close();
    
    if (cnt==0)
        return false;
    else
        return true;
}


//==============================================
// A4964_SPI		
//==============================================
U8 A4964_Demand_Input (U16 DemandInput) 
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r30_DEMAND_INUPT];
    //----------------------------------------------
    // DGS[1:0](b9-b8) Selects output DIAG terminal
    wData &= ~(1023<<1);
    wData |= (DemandInput<<1);
//----------------------------------------------
    stat = A4964_SPI(wData, 0 , &receiveData);

    if (stat)
        regConfig[r30_DEMAND_INUPT] = wData;

    return stat;
}


//==============================================
// A4964_SPI		
//==============================================
U8 A4964_ReadBack (void) 
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r31_READBACK];

    //----------------------------------------------
    stat = A4964_SPI(wData, 0 , &receiveData);

    if (stat)
        regConfig[r31_READBACK] = wData;

    return stat;
}


//==============================================
// A4964_SPI		
//==============================================
U32 A4964_ReadBack_Select (U8 rSel)
{
    U8 stat;
    U16 wData;
    U32 tmp;
    
    
    stat = configDataOutput(rSel);

//--------------------------------------------------------------
// read back
    stat = A4964_ReadBack();
//--------------------------------------------------------------
// Convert Data
    
    receiveData &= 0x07FE;
    receiveData >>= 1;
    
    switch (rSel)
    {
    case RBS_DIAGNOSTIC:
        tmp = receiveData;
        break;
    case RBS_MOTOR_SPEED:
        tmp = receiveData;
        tmp *= 2;       // fE = n �� fRES (0.2Hz)
        tmp *= (60/4);  // fE*60/Pole-pairs (rpm)
        break;
    case RBS_CURRENT:

        break;
    case RBS_VOLTAGE:
        tmp = (U32)receiveData*528/100; // 114*0.0528 = 6.0192V, --> 6.01V
        break;
    case RBS_TEMPERATURE:
        tmp = receiveData;
        break;
    case RBS_DEMAND_INPUT:
        tmp = receiveData;
        break;
    case RBS_PEAK_DUTY_CYCLE:
        tmp = receiveData;
        break;
    case RBS_PHASE_ADVANCE:
        tmp = receiveData;
        break;
    }
    
    return tmp;
}















/*
//==================================================
// A4964_init
//==================================================
void A4964_init2 (void)
{

// General config
    
    //==============================================
    // U8 configSenseMaxThresh(U8 mit)
    //----------------------------------------------
    // -- Sense Amp Maximum Threshold
    // MIT_200MV
    // MIT_100MV
    // MIT_50MV
    // MIT_25MV

    configSenseMaxThresh(MIT_100MV);
    
    //==============================================
    // U8 configCurrentLimit (U16 ns,U8 scale)
    //----------------------------------------------
    // -- Current Limit Blank Time
    // The range of tOCB is 1 us to 6.6 us
    // blank time ns (1000ns~6600ns)
    //----------------------------------------------
    // -- Current Limit Scale
    // n = 0~15
    // scale (0~100)%

    configCurrentLimit(1800, 100); // 

    
    //==============================================
    // U8 configSpeedFlags (U8 dv,U8 df)
    //----------------------------------------------
    // -- Duty Cycle Compensation
    // DV_DISABLED
    // DV_12V
    // DV_24V
    //----------------------------------------------
    // -- Deceleration Factor
    // DF_1
    // DF_2
    // DF_5
    // DF_10
    //------------------------------------------
    configSpeedFlags(DV_DISABLED, DF_1);


    //----------------------------------------------
    // U8 configSpeed (U8 res, U16 lthr, U16 hthr)
    //----------------------------------------------
    // -- Speed Control Resolution
    //  RES    -  LThreshold(min,max) - HThreshold(min,max)
    // SR_0_1HZ -  (0,12)Hz  - (12.7  ,204.7)Hz
    // SR_0_2HZ -  (0,24)Hz  - (25.4  ,409.4)Hz
    // SR_0_4HZ -  (0,48)Hz  - (50.8  ,818.8)Hz
    // SR_0_8HZ -  (0,96)Hz  - (101.6 ,1637.6)Hz
    // SR_1_6HZ -  (0,192)Hz - (203.2 ,3275.2)Hz
    // SR_3_2HZ -  (0,384)Hz - (406.4 ,6550.4)Hz
    //----------------------------------------------
    // -- Underspeed (Low-Speed) Threshold
    // fSL = 8 �� n �� fRES (Hz)
    //----------------------------------------------
    // -- Underspeed (High-Speed) Threshold
    // fSH = [127 + (n �� 128)] �� fRES (Hz)

    configSpeed(SR_0_4HZ, 0, 820);

    //----------------------------------------------
    // U8 configSpeedAccel (U16 accHzX10,U8 gain)
    //----------------------------------------------
    // -- Speed Control Acceleration Limit
    // n = 0~31 , The range of KSL is 6.3 Hz to 204.7 Hz.
    // input range : 63 ~ 2047
    //----------------------------------------------
    // -- Speed Control Gain
    // n = 0~15 ,The range of KS is 1 to 31.
    // input range : 1 ~ 31

    configSpeedAccel(500, 31);

    
    //----------------------------------------------
    // U8 configStopOnFail (U8 sof)
    //----------------------------------------------
    // -- Enable Stop On Fail Select
    // ESF_NOSTOP
    // ESF_STOP
    configStopOnFail(ESF_STOP);

    //----------------------------------------------
    // U8 configVLRvoltage (U8 sof)
    //----------------------------------------------
    // -- Logic Regulator Voltage
    // VLR_3_3V
    // VLR_5V
    configVLRvoltage(VLR_3_3V);
    
    //----------------------------------------------
    // U8 configGateVoltage (U8 vrg)
    //----------------------------------------------
    // -- Gate Drive Regulator Voltage
    // VRG_8V
    // VRG_11V
    configGateVoltage (VRG_11V);
    
    //----------------------------------------------
    // U8 configOperatingMode (U8 opm)
    //----------------------------------------------
    // -- Operating Mode Select
    // OPM_SPI_ONLY
    // OPM_STAND_ALONE
    configOperatingMode(OPM_SPI_ONLY);


    //----------------------------------------------
    // U8 configWakeMode (U8 lwk)
    //----------------------------------------------
    // -- Wake Mode Select
    // LWK_PWM
    // LWK_LIN
    configWakeMode(LWK_PWM);


    //----------------------------------------------
    // U8 configPWMSense (U8 ipi)
    //----------------------------------------------
    // -- PWM Input Sense (when OPM = 1)
    // IPI_ACT_HIGH
    // IPI_ACT_LOW
    configPWMSense(IPI_ACT_HIGH);

    //----------------------------------------------
    // U8 configCurrentLimitEnable (U8 dil)
    //----------------------------------------------
    // DIL(b3) Disable Current Limit (Speed Modes)
    // DIL_ENABLED
    // DIL_DISABLED
    configCurrentLimitEnable(DIL_ENABLED);
    
    
    //----------------------------------------------
    // U8 configMotorMode(U8 mod)
    //----------------------------------------------
    // -- Selects Motor Control Mode
    // CM_CLOSE_LOOP_SPEED
    // CM_CLOSE_LOOP_CURRENT
    // CM_OPEN_LOOP
    configMotorMode(CM_CLOSE_LOOP_SPEED);

    //----------------------------------------------
    // U8 configDriveMode(U8 mod)
    //----------------------------------------------
    // -- Drive Mode Select
    // DRM_SINUSOIDAL
    // DRM_TRAPEZOIDAL
    configDriveMode(DRM_SINUSOIDAL);
    
    //----------------------------------------------
    // U8 configOvermodulation(U8 ovm)
    //----------------------------------------------
    // -- Overmodulation Select
    // OVM_100P
    // OVM_112P
    // OVM_125P
    // OVM_150P
    configOvermodulation(OVM_100P);

// PWM
    //----------------------------------------------
    // U8 configPWM(U8 phase,U8 align,U16 freq)
    //----------------------------------------------
    // -- Modulation Mode Select
    // MOD_PHASE3
    // MOD_PHASE2
    //------------------------------------------
    // -- Bridge PWM Mode Select
    // PMD_ALIGN_CENTER
    // PMD_ALIGN_EDGE
    //------------------------------------------
    // -- Bridge PWM Fixed Period
    // n = 0~63 , This is equivalent to 50 kHz to 14.2 kHz
    // input range : 14000 ~ 50000 hz

    configPWM(MOD_PHASE3, PMD_ALIGN_CENTER, 50000); // hz


    //----------------------------------------------
    // U8 configDither(U8 ditherTime,U8 dwellTime,U8 stepCnt)
    //----------------------------------------------
    // -- PWM Dither Step Period
    // DP_200NS | DP_1000NS
    // DP_400NS | DP_1200NS
    // DP_600NS | DP_1400NS
    // DP_800NS | DP_1600NS
    //------------------------------------------
    // -- PWM Dither Dwell Time
    // DD_1MS
    // DD_2MS
    // DD_5MS
    // DD_10MS
    //------------------------------------------
    // -- PWM Dither Step Count
    // n = 0~15 , Setting DS[3:0] to 0 will disable PWM dither
    //------------------------------------------
    // return spi write success of failed
    //==============================================
    configDither(DP_1600NS, DD_1MS, 10); // 0 steps = dither disabled
    
    
    //----------------------------------------------
    // U8 configBridge(U8 gain, U16 dt)
    //----------------------------------------------
    // -- Sense Amp Gain
    // SA_2_5
    // SA_5
    // SA_10
    // SA_20
    //------------------------------------------
    // -- Dead Time
    // tDEAD = n �� 50 ns
    // n = 0~63 , tDEAD is 100 ns to 3.15 us. 
    // input range : 100 ~ 3150
    configBridge(SA_20, 600); // gain, dead time ns

// Gate
    //----------------------------------------------
    // U8 configGateCurrent(U8 sel, U8 cur1, U8 cur2)
    //----------------------------------------------
    // -- Select Setting Turn on/off current
    // CUR_TURN_ON
    // CUR_TURN_OFF
    //----------------------------------------------
    // -- Turn-On/Off Current 1
    // n = 0~15 ,The range of IR1/IF1 is �V5(5) mA to �V75(75) mA. 0 will set the gate drive to switch mode
    // input range : 0 ~ 75 , 0 : switch mode
    //------------------------------------------
    // IR2[3:0](b4-b1) Turn-On Current 2
    // n = 0~15 ,The range of IR2(IF2) is �V5(5) mA to �V75(75) mA. 0 will set the gate drive to switch mode
    // input range : 0 ~ 75 , 0 : switch mode

    configGateCurrent(CUR_TURN_OFF, 0, 0); // drive turn-off current1, current2: mA, 0=switching
    configGateCurrent(CUR_TURN_ON,  0, 0); // drive turn-on  current1, current2: mA, 0=switching

    //----------------------------------------------
    // U8 configGateSlew(U16 nsOn, U16 nsOff)
    //----------------------------------------------
    // Register 5: Gate Drive Configuration
    //----------------------------------------------
    // -- Slew Control Turn-On Time
    // n = 0~15 ,The range of tRS is 0 ns to 750 ns.
    //------------------------------------------
    // -- Slew Control Turn-Off Time
    // n = 0~15 ,The range of tFS is 0 ns to 750 ns.
    
    configGateSlew(0, 0); // ns




// Current Limiting
    //----------------------------------------------
    // U8 configCurrentLimit (U16 ns,U8 scale)
    //----------------------------------------------
    // -- Current Limit Blank Time
    // The range of tOCB is 1 us to 6.6 us
    // blank time ns (1000ns~6600ns)
    //----------------------------------------------
    // -- Current Limit Scale
    // n = 0~15
    // scale (0~100)%
    //----------------------------------------------
    // return spi write success of failed
    //==============================================
    configCurrentLimit(1000, 100); // blank time ns, scale %

    
// VDS
    //==============================================
    // U8 configVDSOvervoltage(U16 mvX100) -- Overvoltage Threshold
    //----------------------------------------------
    // -- VDS Overvoltage Threshold
    // n = 0~63
    // VDST = 0 ~ 3150mV
    //----------------------------------------------
    configVDSOvervoltage(3150); // mv
    
    
    //----------------------------------------------
    // U8 configVDSFaultQualifier(U8 mod,U16 ns) -- VDS Fault Qualifier 
    //----------------------------------------------
    // -- VDS Fault Qualifier Mode
    // VDQ_DEBOUNCE
    // VDQ_BLANK
    //----------------------------------------------
    // -- VDS Qualify Time
    // n = 0~63
    // tVDQ is 600 ns to 3150 ns

    configVDSFaultQualifier(DEBOUNCE, 3150); // ns

    
// Startup
    //----------------------------------------------
    // U8 cconfigStartupAlign (hold time ms, duty)
    //----------------------------------------------
    // -- Alignment (Hold) Time
    // n = 0~31 , tHOLD is 0 to 3000 ms.
    // input range: 0 ~ 3000
    //----------------------------------------------
    // -- Peak PWM Duty During Alignment
    // n = 0~31 ,  DH is 3.125% to 100%
    // input range: 0 ~ 100

    configStartupAlign(100, 3); // hold time ms, duty cycle percent
    
    
    //----------------------------------------------
    // U8 configStartupMotor(STM_COAST_OFF, RSC_RESTART_OFF, 75, HR_0P)
    //----------------------------------------------
    // Register 16: Startup Configuration
    //----------------------------------------------
    // -- Start Coast Mode Select
    // STM_COAST_OFF
    // STM_COAST_ON
    //----------------------------------------------
    // -- Restart Control
    // RSC_RESTART_OFF
    // RSC_RESTART_ON
    //----------------------------------------------
    // -- Motor Constant (Ratio Between Speed and BEMF)
    // n = 0~15 , KM is 0.3 to 1.05
    // input value = 30~105
    //----------------------------------------------
    // -- Alignment Duty Cycle Ramp Time (Time to reach peak)
    // HR_0P
    // HR_25P
    // HR_50P
    // HR_100P

    configStartupMotor(STM_COAST_OFF, RSC_RESTART_OFF, 75, HR_0P);
    
    //----------------------------------------------
    // U8 configWindmill(DISABLED, 7, 50); // freq hz, duty cycle percent
    //----------------------------------------------
    // -- Windmill Mode Select
    // WIN_DISABLED
    // WIN_ENABLED
    //----------------------------------------------
    // -- Minimum Windmill Detection Frequency
    // n = 0~7 , fWM is 0.4 Hz to 22.8 Hz.
    // input range : 4~228
    //----------------------------------------------
    // -- Duty Cycle During Windmill Braking
    // n = 0~15 , DWB is 6.25% to 100%.
    // input value : 0 ~ 100

    configWindmill(WIN_DISABLED, 7, 50); // freq hz, duty cycle percent




// Ramping
    //------------------------------------------------------
    // configRamp(start freq, start duty, end freq, end duty);
    //------------------------------------------------------
    // -- Start Ramp Initial Frequency
    // n = 0~15 , The range of fS1 is 0.5 Hz to 8 Hz
    // input range: 5~80
    //------------------------------------------------------
    // -- Start Ramp Initial Duty Cycle
    // n = 0~15 , The range of DS1 is 6.25% to 100%
    // input range: 0 ~ 100
    //----------------------------------------------
    // -- Start Ramp Final Frequency
    // n = 0~15 , The range of fS2 is 10 Hz to 47.5 Hz
    // input range: 100~475
    //------------------------------------------------------
    // -- Start Ramp Final Duty Cycle
    // n = 0~15 , The range of DS1 is 6.25% to 100%
    // input range: 0 ~ 100

    configRamp(15, 3,  325, 3);  // start(0.5~8.0) hz, duty(0~100), end freq (10.0~47.5), duty(0~100)

    
    //------------------------------------------------------
    // U8 configRampStep(U16 ms, U8 freq); // step time ms, step freq
    //----------------------------------------------
    // -- Start Ramp Step Time
    // n = 0~15 ,The range of tSTS is 10 ms to 300 ms.
    // intpu ragne : 10 ~ 300
    //----------------------------------------------
    // -- Start Ramp Frequency Step
    // n = 0~15 
    // SFS_0_0125HZ|SFS_1_5HZ
    // SFS_0_025HZ |SFS_2HZ
    // SFS_0_05HZ  |SFS_2_5HZ
    // SFS_0_1HZ   |SFS_3HZ
    // SFS_0_2HZ   |SFS_5HZ
    // SFS_0_4HZ   |SFS_8HZ
    // SFS_0_8HZ   |SFS_10HZ
    // SFS_1HZ     |SFS_15HZ
    // input range : 0 ~ 15

    configRampStep(10, SFS_2HZ); // step time (10 ms to 300 ms.) ms, step freq

    //------------------------------------------------------
    // configPhaseAdvance(mode, gain, degrees);
    //------------------------------------------------------
    // --Phase Advance Mode
    // PAM_MANUAL
    // PAM_AUTO
    //------------------------------------------------------
    // --Auto Phase Advance Control Gain
    // KIP_GAIN_1
    // KIP_GAIN_2
    // KIP_GAIN_4
    // KIP_GAIN_8
    //------------------------------------------------------
    // -- PHASE ADVANCE
    // n = 0 ~ 63 , The range of �cADV is 0 to 43.4�X and 60�X
    // input range : 0 ~ 600
    //------------------------------------------------------
    configPhaseAdvance(PAM_AUTO, KIP_GAIN_4, 0);



// BEMF
    //------------------------------------------------------
    // configBEMFDetectWindow(degreesX10);
    //------------------------------------------------------
    // The range of �cBW is 1.4�X to 43.4�X and 60�X.
    // input range: 0 ~ 600

    configBEMFDetectWindow(56);    // degrees(5.6)*10


    //------------------------------------------------------
    // configBEMF(U8 BS, U8 BF)
    //------------------------------------------------------
    // --BEMF Sampling per cycle
    // BS_SPC_1  
    // BS_SPC_2  
    // BS_SPC_3  
    // BS_SPC_6  
    //------------------------------------------------------
    //--Windmill BEMF Filter Time
    // BF_0     | BF_5MS          
    // BF_200US | BF_6MS   
    // BF_400US | BF_10MS   
    // BF_600US | BF_12MS   
    // BF_800US | BF_14MS   
    // BF_1MS   | BF_16MS   
    // BF_2MS   | BF_18MS   
    // BF_4MS   | BF_20MS   

    configBEMF(BS_SPC_6, BF_10MS); // samplecount, filtertime



    //------------------------------------------------------
    // Commutation control
    //------------------------------------------------------
    // configCommSteadyPow2(KCP, KCI);
    // configCommTransientPow2(KCP, KCI);
    //------------------------------------------------------
    // The range of KCP/KCI is 1/128 to 256. (-7 ~ 8)
    // 2^-7 = 1/128 | 2^0  = 1  | 2^6  = 64
    // 2^-6 = 1/64  | 2^1  = 2  | 2^7  = 128
    // 2^-5 = 1/32  | 2^2  = 4  | 2^8  = 256
    // 2^-4 = 1/16  | 2^3  = 8  |
    // 2^-2 = 1/8   | 2^4  = 16 |
    // 2^-1 = 1/2   | 2^5  = 32 |

    configCommSteadyPow2(0, 0);     // P=2^0=1, I=2^0=1
    configCommTransientPow2(0, 0);  // P=2^0=1, I=2^0=1
    
    
    //----------------------------------------------
    // U8 configDIAG (U8 DGS);
    //----------------------------------------------
    // Register 29: Readback Select
    //----------------------------------------------
    // -- Selects output DIAG terminal
    // DGS_LOW_ON_FAULT
    // DGS_FG
    // DGS_PULSE_HIGH
    // DGS_PULSE_FG

    configDIAG (DGS_LOW_ON_FAULT);
    
    //----------------------------------------------
    // U8 configDataOutput (U8 RBS);
    //----------------------------------------------
    // -- Selects data output on register 31
    // RBS_DIAGNOSTIC
    // RBS_MOTOR_SPEED
    // RBS_CURRENT
    // RBS_VOLTAGE
    // RBS_TEMPERATURE
    // RBS_DEMAND_INPUT
    // RBS_PEAK_DUTY_CYCLE
    // RBS_PHASE_ADVANCE

    configDataOutput (RBS_DIAGNOSTIC);
}
*/






//------------------------------------------------------------------------------
// SYSTEM CONFIG
//------------------------------------------------------------------------------


//==============================================
// U8 configStopOnFail (U8 sof)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// ESF(b9) Enable Stop On Fail Select
// 0: No stop on fail
// 1: Stop on fail
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configStopOnFail (U8 esf)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //------------------------------------------
    // ESF(b9) Enable Stop On Fail Select
    if (esf)
        wData |= (1<<9);
    else
        wData &= ~(1<<9);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}

//==============================================
// U8 configVLRvoltage (U8 sof)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// VLR(b8) Logic Regulator Voltage
// 0: 3.3 V
// 1: 5 V
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configVLRvoltage (U8 vlr)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // VLR(b8) Logic Regulator Voltage
    if (vlr)
        wData |= (1<<8);
    else
        wData &= ~(1<<8);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//==============================================
// U8 configGateVoltage (U8 vrg)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// VRG(b7) Gate Drive Regulator Voltage
// 0: 8 V
// 1: 11 V
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configGateVoltage (U8 vrg)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // VRG(b7) Gate Drive Regulator Voltage
    if (vrg)
        wData |= (1<<7);
    else
        wData &= ~(1<<7);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;
    
    return stat;
}


//==============================================
// U8 configOperatingMode (U8 opm)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// OPM(b6) Operating Mode Select
// 0 SPI only
// 1 Stand-alone with SPI
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configOperatingMode (U8 opm)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // OPM(b6) Operating Mode Select
    if (opm)
        wData |= (1<<6);
    else
        wData &= ~(1<<6);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//==============================================
// U8 configWakeMode (U8 lwk)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// LWK(b5) Wake Mode Select
// 0: PWM Wake Mode
// 1: LIN Wake Mode
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configWakeMode (U8 lwk)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // LWK(b5) Wake Mode Select
    if (lwk)
        wData |= (1<<5);
    else
        wData &= ~(1<<5);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//==============================================
// U8 configPWMSense (U8 ipi)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// IPI(b4) PWM Input Sense (when OPM = 1)
// 0: True, Active high
// 1: Inverted, Active low
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configPWMSense (U8 ipi)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // IPI(b4) PWM Input Sense (when OPM = 1)
    if (ipi)
        wData |= (1<<4);
    else
        wData &= ~(1<<4);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//==============================================
// U8 configCurrentLimitEnable (U8 dil)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// DIL(b3) Disable Current Limit (Speed Modes)
// 0: Enabled
// 1: Disabled
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configCurrentLimitEnable (U8 dil)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // DIL Disable Current Limit (Speed Modes)
    if (dil)
        wData |= (1<<3);
    else
        wData &= ~(1<<3);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//==============================================
// U8 configMotorMode(U8 mod)
//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
// CM[1:0](b1-b0) Selects Motor Control Mode
// 00: Closed-loop speed D
// 01: Closed-loop speed
// 10: Closed-loop current
// 11: Open-loop speed
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configMotorMode (U8 mod)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r00_PWM_CONFIG];
    //----------------------------------------------
    // CM[1:0] Selects Motor Control Mode    
    wData &= ~(3<<1);
    wData |= (mod<<1);

    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//------------------------------------------------------------------------------
// Register 27: Motor Function Control
//------------------------------------------------------------------------------


//==============================================
// U8 configLIN(U8 len)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// LEN(b8) Lin Enable
// 0 Standby
// 1 Active
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configLIN (U8 len)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // LEN(b8) Lin Enable
    if (len)
        wData |= (1<<8);
    else
        wData &= ~(1<<8);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}


//==============================================
// U8 configSLP (U8 slp)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// GTS(b7) Go to Sleep Command
// 0    No change in state D
// 1    No change in state
// 1��0 No change in state
// 0��1 Enter sleep state if enabled
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configSLP (U8 slp)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // GTS(b7) Go to Sleep Command
    if (slp)
        wData |= (1<<7);
    else
        wData &= ~(1<<7);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}

//==============================================
// U8 configOvermodulation(U8 ovm)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// OVM[1:0](b6-b5) Overmodulation Select
// 00: None (100%) D
// 01: 112.5%
// 10: 125%
// 11: 150%
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configOvermodulation (U8 ovm)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // OVM[1:0](b6-b5) Overmodulation Select
    wData &= ~(3<<5);
    wData |= (ovm<<5);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}



//==============================================
// U8 configDriveMode(U8 mod)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// DRM(b4) Drive Mode Select
// 0 Sinusoidal
// 1 Trapezoidal
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configDriveMode (U8 drm)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // DRM(b4) Drive Mode Select
    if (drm)
        wData |= (1<<4);
    else
        wData &= ~(1<<4);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}


//==============================================
// U8 configBRAKE (U8 brk)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// BRK(b3) Brake Function Select
// 0 Brake disabled
// 1 Brake enabled
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configBRAKE (U8 brk)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // BRK Brake Function Select
    if (brk)
        wData |= (1<<3);
    else
        wData &= ~(1<<3);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}


//==============================================
// U8 configDIR (U8 dir)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// DIR(b2) Rotation Direction Select
// 0 Forward 
// 1 Reverse 
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configDIR (U8 dir)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // BRK Brake Function Select
    if (dir)
        wData |= (1<<2);
    else
        wData &= ~(1<<2);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}


//==============================================
// U8 configRUN (U8 run)
//----------------------------------------------
// Register 27: Motor Function Control 
//----------------------------------------------
// RUN(b1) Run enable
// 0 Disable outputs, coast motor
// 1 Start and run motor
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configRUN (U8 run)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r27_MOTORF_UNCTION];
    //----------------------------------------------
    // BRK Brake Function Select
    if (run)
        wData |= (1<<1);
    else
        wData &= ~(1<<1);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r27_MOTORF_UNCTION] = wData;

    return stat;
}



//------------------------------------------------------------------------------
// Register 21: Speed Control Loop Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configSpeedAccel (U16 accHzX10,U8 gain)
//----------------------------------------------
// Register 21: Speed Control Loop Configuration
//----------------------------------------------
// SGL[4:0](b9-b5) Speed Control Acceleration Limit
// KSL = 6.3 + (n �� 6.4) Hz
// n = 0~31 , The range of KSL is 6.3 Hz to 204.7 Hz.
// input range : 63 ~ 2047
//----------------------------------------------
// SG[3:0](b4-b1) Speed Control Gain
// KS= 1 + (n �� 2)
// n = 0~15 ,The range of KS is 1 to 31.
// input range : 1 ~ 31
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configSpeedAccel (U16 accHzX10,U8 gain)
{
    U8 stat;
    U16 nSGL,nSG,wData;

    wData = regConfig[r21_SPEED_ACC];
    //----------------------------------------------
    // SGL[4:0](b9-b5) Speed Control Acceleration Limit
    // KSL = 6.3 + (n �� 6.4) Hz
    // n = 0~31 , The range of KSL is 6.3 Hz to 204.7 Hz.
    if (accHzX10 < 63)
        nSGL = 0;
    else 
        nSGL = (accHzX10-63)/64;
    
    if (nSGL >= 31)
        nSGL = 31;

    wData &= ~(31<<5);
    wData |= (nSGL<<5);
    
    //----------------------------------------------
    // SG[3:0](b4-b1) Speed Control Gain
    // KS= 1 + (n �� 2)
    // n = 0~15 ,The range of KS is 1 to 31.
    if (gain <= 1)
        nSG = 0;
    else
        nSG = (gain-1)/2;
    
    if (nSG >= 15)
        nSG = 15;

    wData &= ~(15<<1);
    wData |= (nSG<<1);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r21_SPEED_ACC] = wData;

    return stat;
}







//------------------------------------------------------------------------------
// Register 22: Speed Control Loop Configuration
// Register 23: Speed Control Loop Configuration
//------------------------------------------------------------------------------


//==============================================
// U8 configSpeedFlags (U8 dv,U8 df)
//----------------------------------------------
// Register 22: Speed Control Loop Configuration
//----------------------------------------------
// DV[1:0](b9-b8) Duty Cycle Compensation
// 00: Disabled
// 01: 12 V
// 10: 24 V
// 11: 12 V
//----------------------------------------------
// DF[1:0](b7-b6) Deceleration Factor
// 00: 1
// 01: 2
// 10: 5
// 11: 10
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configSpeedFlags (U8 dv,U8 df)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r22_SPEED_RESOLUTION];
    //------------------------------------------
    // DV[1:0](b9-b8) Duty Cycle Compensation
    wData &= ~(3<<8);
    wData |= (dv<<8);
    
    //------------------------------------------
    // DF[1:0](b7-b6) Deceleration Factor
    wData &= ~(3<<6);
    wData |= (df<<6);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r22_SPEED_RESOLUTION] = wData;

    return stat;
}



//==============================================
// U8 configSpeed (U8 res, U16 lthr, U16 hthr)
//----------------------------------------------
// Register 22: Speed Control Loop Configuration
//----------------------------------------------
// SR[2:0](b3-b1) Speed Control Resolution
//      RES    -  LThreshold(min,max) - HThreshold(min,max)
// 000: 0.1 Hz -  (0,12)Hz  - (12.7  ,204.7)Hz
// 001: 0.2 Hz -  (0,24)Hz  - (25.4  ,409.4)Hz
// 010: 0.4 Hz -  (0,48)Hz  - (50.8  ,818.8)Hz
// 011: 0.8 Hz -  (0,96)Hz  - (101.6 ,1637.6)Hz
// 100: 1.6 Hz -  (0,192)Hz - (203.2 ,3275.2)Hz
// 101: 3.2 Hz -  (0,384)Hz - (406.4 ,6550.4)Hz
// 110: 3.2 Hz -  (0,384)Hz - (406.4 ,6550.4)Hz
// 111: 3.2 Hz -  (0,384)Hz - (406.4 ,6550.4)Hz
//----------------------------------------------
// Register 23: Speed Control Loop Configuration
//----------------------------------------------
// SL[3:0](b8-b5) Underspeed (Low-Speed) Threshold
// fSL = 8 �� n �� fRES (Hz)
//----------------------------------------------
// SH[3:0](b4-b1) Underspeed (High-Speed) Threshold
// fSH = [127 + (n �� 128)] �� fRES (Hz)
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configSpeed (U8 res, U16 lthr, U16 hthr)
{
    U8 stat,fRES;
    U16 nSL,nSH,wData;

    wData = regConfig[r22_SPEED_RESOLUTION];
    //------------------------------------------
    // SR[2:0](b3-b1) Speed Control Resolution
    wData &= ~(15<<1);
    wData |= (res<<1);
    
//------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r22_SPEED_RESOLUTION] = wData;
    else
        return stat;
    
    
    //------------------------------------------
    wData = regConfig[r23_SPEED_THRESHOLD];
    if (res >= 5)
        res = 5;
    fRES = 2^res;
    //------------------------------------------
    // SL[3:0](b8-b5) Underspeed (Low-Speed) Threshold
    // fSL = 8 �� n �� fRES (Hz)
    nSL = (lthr*10)/fRES/8;
    if (nSL >= 15)
        nSL = 15;
    wData &= ~(15<<5);
    wData |= (nSL<<5);
    //----------------------------------------------
    // SH[3:0](b4-b1) Underspeed (High-Speed) Threshold
    // fSH = [127 + (n �� 128)] �� fRES (Hz)
    nSH = (hthr*10)/fRES/8;
    if (nSH >= 15)
        nSH = 15;
    wData &= ~(15<<1);
    wData |= (nSH<<1);
//------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r23_SPEED_THRESHOLD] = wData;

    return stat;
}




//------------------------------------------------------------------------------
// Register 7: VDS Monitor and Sense Amp Configuration
// Register 8: VDS Fault Qualifier Mode
//------------------------------------------------------------------------------
//==============================================
// U8 configSenseMaxThresh(U8 mit)
//----------------------------------------------
// Register 7: VDS Monitor and Sense Amp Configuration
//----------------------------------------------
// MIT[1:0](b9-b8) Sense Amp Maximum Threshold
// 00: 200 mV D
// 01: 100 mV
// 10: 50 mV
// 11: 25 mV
// input range : 0 ~ 3
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configSenseMaxThresh (U8 mit)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r07_VDS_MAXI_THRESHOLD];
    //------------------------------------------
    // MIT[1:0] Sense Amp Maximum Threshold
    wData &= ~(3<<8);
    wData |= (mit<<8);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r07_VDS_MAXI_THRESHOLD] = wData;

    return stat;
}


//==============================================
// U8 configVDSOvervoltage(U16 mvX100) -- Overvoltage Threshold
//----------------------------------------------
// VT[5:0](b6-b1) VDS Overvoltage Threshold
// VDST = n �� 50 mV
// n = 0~63
// VDST = 0 ~ 3150mV
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configVDSOvervoltage(U16 mv)
{
    U8 stat;
    U16 nVT,wData;
    
    wData = regConfig[r07_VDS_MAXI_THRESHOLD];
    //------------------------------------------
    // VT[5:0](b6-b1) VDS Overvoltage Threshold
    // VDST = n �� 50 mV , The range of VDST is 0 to 3.15 V.
    if (mv >= 3150)
        nVT = 63;
    else
        nVT = mv/50;
    
    wData &= ~(63<<1);
    wData |= (nVT<<1);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r07_VDS_MAXI_THRESHOLD] = wData;

    return stat;
}


//==============================================
// U8 configVDSFaultQualifier(U8 mod,U16 ns) -- VDS Fault Qualifier 
//----------------------------------------------
// VDQ(b8) VDS Fault Qualifier Mode
// 0:Debounce
// 1:Blank
//----------------------------------------------
// VQT[5:0](b6-b1) VDS Qualify Time
// tVDQ = n �� 50 ns
// n = 0~63
// tVDQ is 600 ns to 3150 ns
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configVDSFaultQualifier(U8 mod,U16 ns)
{
    U8 stat;
    U16 nVQT,wData;
    
    wData = regConfig[r08_VDS_FAULT];
    //------------------------------------------
    // VDQ(b8) VDS Fault Qualifier Mode
    if (mod == VDQ_DEBOUNCE)
        wData &= ~(1<<8);
    else
        wData |=  (1<<8);
    //----------------------------------------------
    // VQT[5:0](b6-b1) VDS Qualify Time
    // tVDQ = n �� 50 ns
    // n = 0~63 ,  tVDQ is 600 ns to 3150 ns
    if (ns <= 600)
        nVQT = 0;
    else
        nVQT = ns/50;
    
    wData &= ~(63<<1);
    wData |= (nVQT<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r08_VDS_FAULT] = wData;

    return stat;
}









//------------------------------------------------------------------------------
// Register 0: PWM Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configPWM(U8 phase,U8 align,U16 freq)
//----------------------------------------------
// Register 0: PWM Configuration
//----------------------------------------------
// MOD(b9) Modulation Mode Select
// 0: 3-phase D
// 1: 2-phase
//------------------------------------------
// PMD(b7) Bridge PWM Mode Select
// 0: Center aligned D
// 1: Edge aligned
//------------------------------------------
// PW[5:0](b6-b1) Bridge PWM Fixed Period
// tPW = 20.10 �gs + (n �� 0.8 �gs) when PMD = 0
// tPW = 20.05 �gs + (n �� 0.8 �gs) when PMD = 1
// n = 0~63 , This is equivalent to 50 kHz to 14.2 kHz
// input range : 14000 ~ 50000
//------------------------------------------
// scale (0~100)%
// return spi write success of failed
//==============================================
U8 configPWM(U8 mod,U8 pmd,U16 freq)
{
    U8 stat;
    U16 nPW,wData;
    U32 tmp;

    wData = regConfig[r00_PWM_CONFIG];
    //------------------------------------------
    // MOD(b9) Modulation Mode Select
    if (mod == MOD_PHASE3)
        wData &= ~(1<<9);
    else
        wData |= (1<<9);
    //------------------------------------------
    // PMD(b7) Bridge PWM Mode Select
    if (pmd == PMD_ALIGN_CENTER)
        wData &= ~(1<<7);
    else
        wData |= (1<<7);   
    //------------------------------------------
    // PW[5:0](b6-b1) Bridge PWM Fixed Period
    // tPW = 20.10 �gs + (n �� 0.8 �gs) when PMD = 0
    // tPW = 20.05 �gs + (n �� 0.8 �gs) when PMD = 1
    // n = 0~63 , This is equivalent to 50 kHz to 14.2 kHz
    tmp = (U32)100000000L/(U32)freq;
    if (pmd == 0)
    {
        if (tmp <= 2010)
            nPW = 0;
        else
            nPW = (tmp-2010)/80;
    }
    else
    {
        if (tmp <= 2005)
            nPW = 0;
        else
            nPW = (tmp-2005)/80;
    }
    if (nPW >= 63)
        nPW = 63;

    wData &= ~(63<<1);
    wData |= (nPW<<1);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r00_PWM_CONFIG] = wData;

    return stat;
}


//------------------------------------------------------------------------------
// Register 1: PWM Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configDither(U8 ditherTime,U8 dwellTime,U8 stepCnt)
//----------------------------------------------
// Register 1: PWM Configuration
//----------------------------------------------
// DP[2:0](b9-b7) PWM Dither Step Period
// tPW = �V0.2 �gs �V (n �� 0.2 �gs)
// n = 0~7 ,The range of t?PW is �V0.2 us to �V1.6 us
//------------------------------------------
// DD[1:0](b6-b5) PWM Dither Dwell Time
// 00: 1 ms
// 01: 2 ms
// 10: 5 ms
// 11: 10 ms
//------------------------------------------
// DS[3:0](b4-b1) PWM Dither Step Count
// n = 0~15 , Setting DS[3:0] to 0 will disable PWM dither
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configDither(U8 ditherTime,U8 dwellTime,U8 stepCnt)
{
    U8 stat;
    U16 nPW,wData;
    U32 tmp;

    wData = regConfig[r01_PWM_DITHER];
    //------------------------------------------
    // DP[2:0](b9-b7) PWM Dither Step Period
    wData &= ~(7<<7);
    wData |= (ditherTime<<7);
    //------------------------------------------
    // DD[1:0](b6-b5) PWM Dither Dwell Time
    wData &= ~(3<<5);
    wData |= (dwellTime<<5);
    //------------------------------------------
    // DS[3:0](b4-b1) PWM Dither Step Count
    // n = 0~15 , Setting DS[3:0] to 0 will disable PWM dither
    wData &= ~(15<<1);
    wData |= (stepCnt<<1);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r01_PWM_DITHER] = wData;

    return stat;
}



//------------------------------------------------------------------------------
// Register 2: Bridge and Sense Amp Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configBridge(U8 gain, U16 dt)
//----------------------------------------------
// Register 2: Bridge and Sense Amp Configuration
//----------------------------------------------
// SA[1:0](b8-b7) Sense Amp Gain
// 00: 2.5
// 01: 5
// 10: 10
// 11: 20
//------------------------------------------
// DT[5:0](b6-b1) Dead Time
// tDEAD = n �� 50 ns
// n = 0~63 , tDEAD is 100 ns to 3.15 ?s. 
//------------------------------------------
// return spi write success of failed
//==============================================
U8 configBridge(U8 gain, U16 dt)
{
    U8 stat;
    U16 nDT,wData;

    wData = regConfig[r02_BRIDGE_CONFIG];
    //----------------------------------------------
    // SA[1:0](b8-b7) Sense Amp Gain
    wData &= ~(3<<7);
    wData |= (gain<<7);
    //------------------------------------------
    // DT[5:0](b6-b1) Dead Time
    // tDEAD = n �� 50 ns
    // n = 0~63 , tDEAD is 100 ns to 3.15 ?s. 
    nDT = dt/50;
    if (nDT >= 63)
        nDT = 63;
    wData &= ~(63<<1);
    wData |= (nDT<<1);
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r02_BRIDGE_CONFIG] = wData;

    return stat;
}




//------------------------------------------------------------------------------
// Register 3: Gate Drive Configuration
// Register 4: Gate Drive Configuration
// Register 5: Gate Drive Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configGateCurrent(U8 sel, U8 cur1, U8 cur2)
//----------------------------------------------
// Register 3: Gate Drive Configuration
//----------------------------------------------
// IR1[3:0](b8-b5) Turn-On Current 1
// IR1 = n �� �V5 mA
// n = 0~15 ,The range of IR1 is �V5 mA to �V75 mA. 0 will set the gate drive to switch mode
//------------------------------------------
// IR2[3:0](b4-b1) Turn-On Current 2
// IR1 = n �� �V5 mA
// n = 0~15 ,The range of IR1 is �V5 mA to �V75 mA. 0 will set the gate drive to switch mode
//----------------------------------------------
// Register 4: Gate Drive Configuration
//----------------------------------------------
// IF1[3:0](b8-b5) Turn-Off Current 1
// IF1 = n �� 5 mA
// The range of IF1 is 5 mA to 75 mA.
// n = 0~15 ,The range of IF1 is 5 mA to 75 mA. 0 will set the gate drive to switch mode
//----------------------------------------------
// IF2[3:0](b4-b1) Turn-Off Current 2
// IF2 = n �� 5 mA
// The range of IF2 is 5 mA to 75 mA.
// n = 0~15 ,The range of IF2 is 5 mA to 75 mA. 0 will set the gate drive to switch mode
//----------------------------------------------
// scale (0~100)%
// return spi write success of failed
//==============================================
U8 configGateCurrent(U8 gate, U8 cur1, U8 cur2)
{
    U8 stat,regAddr;
    U16 wData;
    
    if (gate == CUR_TURN_ON)
        regAddr = r03_GATE_CURRENT_TON;
    else
        regAddr = r04_GATE_CURRENT_TOFF;

    wData = regConfig[regAddr];
    
    cur1 /= 5;
    if (cur1 >= 15)
        cur1 = 15;
    cur2 /= 5;
    if (cur2 >= 15)
        cur2 = 15;
    wData &= ~(255<<1);
    wData |= ((cur1<<5)|(cur2<<1));
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[regAddr] = wData;

    return stat;
}


//==============================================
// U8 configGateSlew(U16 nsOn, U16 nsOff)
//----------------------------------------------
// Register 5: Gate Drive Configuration
//----------------------------------------------
// TRS[3:0](b9-b5) Slew Control Turn-On Time
// tRS = n �� 50 ns
// n = 0~15 ,The range of tRS is 0 ns to 750 ns.
//------------------------------------------
// TFS[3:0](b4-b1) Slew Control Turn-Off Time
// tFS = n �� 50 ns
// n = 0~15 ,The range of tFS is 0 ns to 750 ns.
//------------------------------------------
// scale (0~100)%
// return spi write success of failed
//==============================================
U8 configGateSlew(U16 nsOn, U16 nsOff)
{
    U8 stat;
    U16 wData;

    wData = regConfig[r05_GATE_CONTROL_TSLEW];

    nsOn /= 50;
    if (nsOn >= 15)
        nsOn = 15;
    nsOff /= 50;
    if (nsOff >= 15)
        nsOff = 15;
    wData &= ~(255<<1);
    wData |= ((nsOn<<5)|(nsOff<<1));
    
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r05_GATE_CONTROL_TSLEW] = wData;

    return stat;
}


//------------------------------------------------------------------------------
// Register 6: Current Limit Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configCurrentLimit (U16 ns,U8 scale)
//----------------------------------------------
// OBT[4:0](b9-b5) Current Limit Blank Time
// tOCB = (n + 2) �� 200 ns,0, 1, 2, and 3 will set the blank time to 1 us.
// The range of tOCB is 1 us to 6.6 us
// blank time ns (1000ns~6600ns)
//----------------------------------------------
// VIL[3:0](b4-b1) Current Limit Scale
// n = 0~15
// scale (0~100)%
//----------------------------------------------
// return spi write success of failed
//==============================================
U8 configCurrentLimit (U16 ns,U8 scale)
{
    U8 stat;
    U16 nOBT,nVIL,wData;
    
    wData = regConfig[r06_CURRENT_LIMIT];
    //------------------------------------------
    // OBT[4:0](b9-b5) Current Limit Blank Time
    // tOCB = (n + 2) �� 200 ns,0, 1, 2, and 3 will set the blank time to 1 ?s.
    if (ns<=1000)
        nOBT = 0;
    else
        nOBT = (ns/200)-2;
    wData &= ~(31<<5);
    wData |= (nOBT<<5);
    //------------------------------------------
    // VIL[3:0] Current Limit Scale
    if (scale>=100)
        nVIL = 15;
    else
        nVIL = (scale<<2)/25;    // 75%*16/100 = 12
    wData &= ~(15<<1);
    wData |= (nVIL<<1);
    //------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r06_CURRENT_LIMIT] = wData;

    return stat;
}




//------------------------------------------------------------------------------
// Register 15: Startup Configuration
// Register 16: Startup Configuration
// Register 17: Startup Configuration
//------------------------------------------------------------------------------


//==============================================
// U8 cconfigStartupAlign(100, 3)
//----------------------------------------------
// Register 15: Startup Configuration
//----------------------------------------------
// HT[3:0](b9-b6) Alignment (Hold) Time
// tHOLD = n �� 200 ms
// n = 0~31 , tHOLD is 0 to 3000 ms.
// input range: 0 ~ 3000
//----------------------------------------------
// HD[4:0](b5-b1) Peak PWM Duty During Alignment
// DH = (n + 1) �� 3.125%
// n = 0~31 ,  DH is 3.125% to 100%
// input range: 0 ~ 100
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configStartupAlign(U16 ms,U8 dutyp)
{
    U8 stat;
    U16 nHT,nHD,wData;
    
    wData = regConfig[r15_STARTUP_ALIGNMENT];
    //------------------------------------------
    // HT[3:0](b9-b6) Alignment (Hold) Time
    // tHOLD = n �� 200 ms
    // n = 0~15 , tHOLD is 0 to 3000 ms.
    if (ms >= 3000)
        ms = 3000;
    nHT = ms/200;
    
    wData &= ~(15<<6);
    wData |= (nHT<<6);
    
    //----------------------------------------------
    // HD[4:0](b5-b1) Peak PWM Duty During Alignment
    // DH = (n + 1) �� 3.125%
    // n = 0~31 ,  DH is 3.125% to 100%
    if (dutyp >= 100)
        nHD = 31;
    else if (dutyp <= 3)
        nHD = 0;
    else
        nHD = ((U32)dutyp*1000/3125)-1;
    
    wData &= ~(31<<1);
    wData |= (nHD<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r15_STARTUP_ALIGNMENT] = wData;

    return stat;
}


//==============================================
// U8 configStartupMotor(COAST_OFF, DESYNC_RESTART_OFF, 750, RAMP_0P);
//----------------------------------------------
// Register 16: Startup Configuration
//----------------------------------------------
// STM(b9) Start Coast Mode Select
// 0: Coast Disable
// 1: Coast Enable
//----------------------------------------------
// RSC(b8) Restart Control
// 0: No restart
// 1: Allow restart after loss of sync
//----------------------------------------------
// KM[3:0](b6-b3) Motor Constant (Ratio Between Speed and BEMF)
// KM = 0.3 + (n �� 0.05)
// n = 0~15 , KM is 0.3 to 1.05
// input value = 30~105
//----------------------------------------------
// HR[1:0] Alignment Duty Cycle Ramp Time 
// (Time to reach peak)
// 00: 0 
// 01: 25% tHOLD
// 10: 50% tHOLD
// 11: 100% tHOLD
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configStartupMotor(U8 coast,U8 rsc,U16 km,U8 rampt)
{
    U8 stat;
    U16 nKM,wData;
    
    wData = regConfig[r16_STARTUP_MODE];
    //----------------------------------------------
    // STM(b9) Start Coast Mode Select
    if (coast)
        wData |= (1<<9);
    else
        wData &= ~(1<<9);
    //----------------------------------------------
    // RSC(b8) Restart Control
    if (rsc)
        wData |= (1<<8);
    else
        wData &= ~(1<<8);
    //----------------------------------------------
    // KM[3:0](b6-b3) Motor Constant (Ratio Between Speed and BEMF)
    // KM = 0.3 + (n �� 0.05)
    // n = 0~15 , KM is 0.3 to 1.05
    // input value = 30~105
    if (km >= 105)
        nKM = 15;
    else if (km < 30)
        nKM = 0;
    else
        nKM = (km-30)/5;
        
    wData &= ~(15<<3);
    wData |= (nKM<<3);
    
    //----------------------------------------------
    // HR[1:0] Alignment Duty Cycle Ramp Time 
    wData &= ~(3<<1);
    wData |= (rampt<<1);
    
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r16_STARTUP_MODE] = wData;

    return stat;
}


//==============================================
// U8 configWindmill(DISABLED, 7, 50); // freq hz, duty cycle percent
//----------------------------------------------
// Register 17: Startup Configuration
//----------------------------------------------
// WIN(b8) Windmill Mode Select
// 0: Windmilling disabled
// 1: Windmilling enabled
//----------------------------------------------
// WMF[2:0](b7-b5) Minimum Windmill Detection Frequency
// fWM = 0.4 + (n �� 3.2) Hz
// n = 0~7 , fWM is 0.4 Hz to 22.8 Hz.
// input range : 4~228
//----------------------------------------------
// WBD[3:0](b4-b1) Duty Cycle During Windmill Braking
// DWB = (n + 1) �� 6.25%
// n = 0~15 , DWB is 6.25% to 100%.
// input value : 0 ~ 100
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configWindmill(U8 mod,U8 freq,U8 dutyp)
{
    U8 stat;
    U16 nWMF,nWBD,wData,tmp;
    
    wData = regConfig[r17_WINDMILL_MODE];
    //----------------------------------------------
    // WIN(b8) Windmill Mode Select
    if (mod)
        wData |= (1<<8);
    else
        wData &= ~(1<<8);
    //----------------------------------------------
    // WMF[2:0](b7-b5) Minimum Windmill Detection Frequency
    // fWM = 0.4 + (n �� 3.2) Hz
    // n = 0~7 , fWM is 0.4 Hz to 22.8 Hz.
    // input range : 4~228
    
    if (freq <= 4)
        nWMF = 0;
    else if (freq >= 228)
        nWMF = 7;
    else
        nWMF = (freq-4)/32;
    
    wData &= ~(7<<5);
    wData |= (nWMF<<5);
    //----------------------------------------------
    // WBD[3:0](b4-b1) Duty Cycle During Windmill Braking
    // DWB = (n + 1) �� 6.25%
    // n = 0~15 , DWB is 6.25% to 100%.
    // input value : 0 ~ 100
    tmp = (U16)dutyp*100;
    if (tmp < 625)
        nWBD = 0;
    else
        nWBD = (tmp/625)-1;
    
    wData &= ~(15<<1);
    wData |= (nWBD<<1);
    
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r17_WINDMILL_MODE] = wData;

    return stat;
}





//------------------------------------------------------------------------------
// Register 18: Start Ramp Initial/Final Frequency
// Register 19: Start Ramp Initial/Final Duty Cycle
// Register 20: Start Ramp Step Time
//------------------------------------------------------------------------------

//==============================================
// U8 configRamp(U16 startFreqX10,U8 startDuty,U16 endFreqX10, U8 endDuty); // start freqx10 hz, duty, end freqx10, duty
//----------------------------------------------
// Register 18: Startup Configuration
//----------------------------------------------
// SF2[3:0](b8-b5) Start Ramp Final Frequency
// fS2 = 10 + (n �� 2.5) Hz
// n = 0~15 , The range of fS2 is 10 Hz to 47.5 Hz
// input range: 100~475
//----------------------------------------------
// SF1[3:0](b4-b1) Start Ramp Initial Frequency
// fS1 = 0.5 + (n �� 0.5) Hz
// n = 0~15 , The range of fS1 is 0.5 Hz to 8 Hz
// input range: 5~80
//----------------------------------------------
// Register 19: Startup Configuration
//----------------------------------------------
// SD2[3:0](b8-b5) Start Ramp Final Duty Cycle
// DS2 = (n + 1) �� 6.25%
// n = 0~15 , The range of DS2 is 6.25% to 100%
// input range: 0 ~ 100
//----------------------------------------------
// SD1[3:0](b4-b1) Start Ramp Initial Duty Cycle
// DS1 = (n + 1) �� 6.25%
// n = 0~15 , The range of DS1 is 6.25% to 100%
// input range: 0 ~ 100
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configRamp(U16 startFreqX10,U8 startDuty,U16 endFreqX10, U8 endDuty)
{
    U8 stat;
    U16 nSF2,nSF1,nSD2,nSD1,wData,tmp;
    
    wData = regConfig[r18_RAMP_FREQ];
    //----------------------------------------------
    // SF2[3:0](b8-b5) Start Ramp Final Frequency
    // fS2 = 10 + (n �� 2.5) Hz
    // n = 0~15 , The range of fS2 is 10.0 Hz to 47.5 Hz
    if (endFreqX10 <= 100)
        nSF2 = 0;
    else if (endFreqX10 >= 475)
        nSF2 = 15;
    else
        nSF2 = (endFreqX10-100)/25;
    
    wData &= ~(15<<5);
    wData |= (nSF2<<5);
    //----------------------------------------------
    // SF1[3:0](b4-b1) Start Ramp Initial Frequency
    // fS1 = 0.5 + (n �� 0.5) Hz
    // n = 0~15 , The range of fS1 is 0.5 Hz to 8 Hz
    if (startFreqX10 <= 5)
        nSF1 = 0;
    else if (startFreqX10 >= 80)
        nSF1 = 15;
    else
        nSF1 = (startFreqX10-5)/5;

    wData &= ~(15<<1);
    wData |= (nSF1<<1);
    
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r18_RAMP_FREQ] = wData;
    else
        return stat;

 //-----------------------------------------------------
    wData = regConfig[r19_RAMP_DUTY];
    //----------------------------------------------
    // SD2[3:0](b8-b5) Start Ramp Final Duty Cycle
    // DS2 = (n + 1) �� 6.25%
    // n = 0~15 , The range of DS2 is 6.25% to 100%
    tmp = (U16)endDuty*100;
    
    if (tmp <= 625)
        nSD2 = 0;
    else if (tmp >= 10000)
        nSD2 = 15;
    else
        nSD2 = (tmp/625)-1;

    wData &= ~(15<<5);
    wData |= (nSD2<<5);
    
    //----------------------------------------------
    // SD1[3:0](b4-b1) Start Ramp Initial Duty Cycle
    // DS1 = (n + 1) �� 6.25%
    // n = 0~15 , The range of DS1 is 6.25% to 100%
    tmp = (U16)startDuty*100;
    
    if (tmp <= 625)
        nSD1 = 0;
    else if (tmp >= 10000)
        nSD1 = 15;
    else
        nSD1 = (tmp/625)-1;

    wData &= ~(15<<1);
    wData |= (nSD1<<1);
    
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r19_RAMP_DUTY] = wData;

    return stat;
}


//==============================================
// U8 configRampStep(U16 ms, U8 freq); // step time ms, step freq
//----------------------------------------------
// Register 20: Startup Configuration
//----------------------------------------------
// STS[3:0](b8-b5) Start Ramp Step Time
// tSS = 10 ms     |n=0
// tSS = n �� 20 ms |n=1..15 , 
// n = 0~15 ,The range of tSTS is 10 ms to 300 ms.
// intpu ragne : 10 ~ 300
//----------------------------------------------
// SFS[3:0](b4-b1) Start Ramp Frequency Step
// n = 0~15 
// 0000 0.0125 Hz|1000 1.5 Hz
// 0001 0.025 Hz |1001 2 Hz
// 0010 0.05 Hz  |1010 2.5 Hz
// 0011 0.1 Hz   |1011 3 Hz
// 0100 0.2 Hz   |1100 5 Hz
// 0101 0.4 Hz   |1101 8 Hz
// 0110 0.8 Hz   |1110 10 Hz
// 0111 1 Hz     |1111 15 Hz
// input range : 0 ~ 15
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configRampStep(U16 stepms, U8 stepfreq)
{
    U8 stat;
    U16 nSTS,nSFS,wData;
    
    wData = regConfig[r20_RAMP_STEP];
    //----------------------------------------------
    // STS[3:0](b8-b5) Start Ramp Step Time
    // tSS = 10 ms     |n=0
    // tSS = n �� 20 ms |n=1..15 , 
    // n = 0~15 ,The range of tSTS is 10 ms to 300 ms.
    if (stepms <= 10)
        nSTS = 0;
    else if (stepms >= 300)
        nSTS = 15;
    else
        nSTS = stepms/20;
    
    wData &= ~(15<<5);
    wData |= (nSTS<<5); 
    
    //----------------------------------------------
    // SFS[3:0](b4-b1) Start Ramp Frequency Step
    nSFS = stepfreq;
    wData &= ~(15<<1);
    wData |= (nSFS<<1);
    
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r20_RAMP_STEP] = wData;

    return stat;
}





//------------------------------------------------------------------------------
// Register 13: BEMF Detection Window
// Register 14: BEMF Sampling
//------------------------------------------------------------------------------

//==============================================
// U8 configBEMFDetectWindow(U16 degX10);
//----------------------------------------------
// BW[4:0](b5-b1) BEMF Detection Window
// �cBW = (n + 1) �� 1.4�X(elec) |n=0..30
// �cBW = 60�X(elec)            |n=31
// n = 0~31 , The range of �cBW is 1.4�X to 43.4�X and 60�X.
// input range: 0 ~ 600
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configBEMFDetectWindow(U16 degX10)
{
    U8 stat;
    U16 nBW,wData;
    
    wData = regConfig[r13_BEMF_WINDOW];
    //----------------------------------------------
    // BW[4:0](b5-b1) BEMF Detection Window
    // �cBW = (n + 1) �� 1.4�X(elec) |n=0..30
    // �cBW = 60�X(elec)            |n=31
    // n = 0~31 , The range of �cBW is 1.4�X to 43.4�X and 60�X.
    if (degX10 >= 434)
        nBW = 31;
    else
        nBW = (degX10/14)-1;
    
    wData &= ~(31<<1);
    wData |= (nBW<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r13_BEMF_WINDOW] = wData;

    return stat;
}


//==============================================
// U8 configBEMF(U8 samplecnt, U8 filtertime); // mode, gain, degrees*10
//----------------------------------------------
// Register 14: BEMF Sampling
//----------------------------------------------
// BS[1:0](b6-b5) BEMF Sampling
// 00: 1
// 01: 2
// 10: 3
// 11: 6
//----------------------------------------------
// BF[3:0](b4-b1) Windmill BEMF Filter Time
// n = 0~15
// 0000: 0      | 1000: 5 ms  
// 0001: 200 ns | 1001: 6 ms  
// 0010: 400 ns | 1010: 10 ms
// 0011: 600 ns | 1011: 12 ms
// 0100: 800 ns | 1100: 14 ms
// 0101: 1 ms   | 1101: 16 ms
// 0110: 2 ms   | 1110: 18 ms
// 0111: 4 ms   | 1111: 20 ms
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configBEMF(U8 samplecnt, U8 filtertime)
{
    U8 stat;
    U16 nPA,wData;
    
    wData = regConfig[r14_BEMF_FILTER];
    //----------------------------------------------
    // BS[1:0](b6-b5) BEMF Sampling
    wData &= ~(3<<5);
    wData |= (samplecnt<<5);   
    //----------------------------------------------
    // BF[3:0](b4-b1) Windmill BEMF Filter Time
    wData &= ~(15<<1);
    wData |= (filtertime<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r14_BEMF_FILTER] = wData;

    return stat;
}




//------------------------------------------------------------------------------
// Register 11: Steady-State Commutation Configuration
// Register 12: Transient Commutation Configuration
//------------------------------------------------------------------------------

//==============================================
// U8 configCommSteadyPow2(S8 np,S8 ni);
//----------------------------------------------
// Register 11: Steady-State Commutation Configuration
//----------------------------------------------
// CP[3:0](b8-b5) Steady-State Commutation Controller Proportional Gain
// KCP = 2^(n-7)
// n = 0~15 ,The range of KCP is 1/128 to 256.
// input range : -7 ~ 8
//----------------------------------------------
// CI[3:0](b4-b1) Steady-State Commutation Controller Integral Gain
// KCI = 2^(n-7)
// n = 0~15 ,The range of KCI is 1/128 to 256.
// input range : -7 ~ 8
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configCommSteadyPow2(S8 np,S8 ni)
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r11_COMMUTATION_STEADY_PI];
   //----------------------------------------------
    // CP[3:0](b8-b5) Steady-State Commutation Controller Proportional Gain
    // KCP = 2^(n-7)
    // n = 0~15 ,The range of KCP is 1/128 to 256.
    if (np < -7)
        np = -7;
    else if (np >= 8)
        np = 8;
    np += 7;
    
    wData &= ~(15<<5);
    wData |= (np<<5);
    
    //----------------------------------------------
    // CI[3:0](b4-b1) Steady-State Commutation Controller Integral Gain
    // KCI = 2^(n-7)
    // n = 0~15 ,The range of KCI is 1/128 to 256.
    if (ni < -7)
        ni = -7;
    else if (ni >= 8)
        ni = 8;
    ni += 7;
    
    wData &= ~(15<<1);
    wData |= (ni<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r11_COMMUTATION_STEADY_PI] = wData;

    return stat;
}



//==============================================
// U8 configCommTransientPow2(S8 np,S8 ni);
//----------------------------------------------
// Register 12: Transient Commutation Configuration
//----------------------------------------------
// CP[3:0](b8-b5) Transient Commutation Controller Proportional Gain
// KCP = 2^(n-7)
// n = 0~15 ,The range of KCP is 1/128 to 256.
//----------------------------------------------
// CI[3:0](b4-b1) Transient Commutation Controller Integral Gain
// KCI = 2^(n-7)
// n = 0~15 ,The range of KCI is 1/128 to 256.
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configCommTransientPow2(S8 np,S8 ni)
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r12_COMMUTATION_TRANSIENT_PI];
   //----------------------------------------------
    // CP[3:0](b8-b5) Steady-State Commutation Controller Proportional Gain
    // KCP = 2^(n-7)
    // n = 0~15 ,The range of KCP is 1/128 to 256.
    if (np < -7)
        np = -7;
    else if (np >= 8)
        np = 8;
    np += 7;
    
    wData &= ~(15<<5);
    wData |= (np<<5);
    
    //----------------------------------------------
    // CI[3:0](b4-b1) Steady-State Commutation Controller Integral Gain
    // KCI = 2^(n-7)
    // n = 0~15 ,The range of KCI is 1/128 to 256.
    if (ni < -7)
        ni = -7;
    else if (ni >= 8)
        ni = 8;
    ni += 7;
    
    wData &= ~(15<<1);
    wData |= (ni<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r12_COMMUTATION_TRANSIENT_PI] = wData;

    return stat;
}



//==============================================
// U8 configPhaseAdvance(U8 mod, U8 gain,U16 degX10); // mode, gain, degrees*10
//----------------------------------------------
// Register 26: Phase Advance Select
//----------------------------------------------
// PAM(b9) Phase Advance Mode
// 0 Manual
// 1 Automatic
//----------------------------------------------
// KIP[1:0](b8-b7) Auto Phase Advance Control Gain
// n = 0~3
// 00 1 
// 01 2
// 10 4
// 11 8
//----------------------------------------------
// PA[5:0](b6-b1) Phase Advance
// �cADV = n �� 0.7�X(elec) |n=0..62
// �cADV = 60�X(elec)      |n=63
// n = 0 ~ 63 , The range of �cADV is 0 to 43.4�X and 60�X
// input range : 0 ~ 600
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configPhaseAdvance(U8 mod, U8 gain,U16 degX10)
{
    U8 stat;
    U16 nPA,wData;
    
    wData = regConfig[r26_PHASE_ADVANCE];
    //----------------------------------------------
    // PAM(b9) Phase Advance Mode
    // 0 Manual
    // 1 Automatic
    if (mod)
        wData |= (1<<9);
    else
        wData &= ~(1<<9);    
    //----------------------------------------------
    // KIP[1:0](b8-b7) Auto Phase Advance Control Gain
    wData &= ~(3<<7);
    wData |= (gain<<7);
    //----------------------------------------------
    // PA[5:0](b6-b1) Phase Advance
    // �cADV = n �� 0.7�X(elec) |n=0..62
    // �cADV = 60�X(elec)      |n=63
    // n = 0 ~ 63 , The range of �cADV is 0 to 43.4�X and 60�X
    if (degX10 >= 434)
        nPA = 63;
    else
        nPA = degX10/7;
    
    wData &= ~(63<<1);
    wData |= (nPA<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r26_PHASE_ADVANCE] = wData;

    return stat;
}

//------------------------------------------------------------------------------
// Register 29: Readback Select
//------------------------------------------------------------------------------

//==============================================
// U8 configDIAG (U8 DGS);
//----------------------------------------------
// Register 29: Readback Select
//----------------------------------------------
// DGS[1:0](b9-b8) Selects output DIAG terminal
// 00: Active low fault flag D
// 01: FG; high when motor is stationary
// 10: Pulse output; high when no fault present
// 11: Pulse output; FG when no fault present
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configDIAG (U8 DGS)
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r29_READBACK_SELECT];
    //----------------------------------------------
    // DGS[1:0](b9-b8) Selects output DIAG terminal
    wData &= ~(3<<8);
    wData |= (DGS<<8);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r29_READBACK_SELECT] = wData;

    return stat;
}

//==============================================
// U8 configDataOutput (U8 RBS);
//----------------------------------------------
// Register 29: Readback Select
//----------------------------------------------
// RBS[2:0](b2-b1) Selects data output on register 31
// 000: Diagnostic register D
// 001: Motor speed
// 010: Average supply current
// 011: Supply voltage
// 100: Chip temperature
// 101: Demand input
// 110: Applied bridge peak duty cycle
// 111: Applied phase advance
//----------------------------------------------
// return : 
// 1: write success
// 0: write failed
//==============================================
U8 configDataOutput (U8 RBS)
{
    U8 stat;
    U16 wData;
    
    wData = regConfig[r29_READBACK_SELECT];
    //----------------------------------------------
    // RBS[2:0](b2-b1) Selects data output on register 31
    wData &= ~(7<<1);
    wData |= (RBS<<1);
//----------------------------------------------
    stat = A4964_SPI(wData,cmdWR,&receiveData);

    if (stat)
        regConfig[r29_READBACK_SELECT] = wData;

    return stat;
}






