/* 
 * File    A964_def.h
 * Author  jeffyet
 *
 * Created on May 13, 2021, 10 02 AM
 */

#ifndef A4964_DEF_H
#define	A4964_DEF_H

#include "system.h"
//extern void A4964_init (void);
//extern U8 A4964_SPI(U16 Data,U8 WR);

//------------------------------------------------------------------------------
// r/w bit define
#define cmdRD   0
#define cmdWR   1


#define   r00_PWM_CONFIG          0
#define   r01_PWM_DITHER          1
#define   r02_BRIDGE_CONFIG       2
#define   r03_GATE_CURRENT_TON    3
#define   r04_GATE_CURRENT_TOFF   4
#define   r05_GATE_CONTROL_TSLEW  5
#define   r06_CURRENT_LIMIT       6
#define   r07_VDS_MAXI_THRESHOLD  7
#define   r08_VDS_FAULT           8
#define   r09_WATCHDOG_PERIOD     9
#define   r10_WATCHDOG_WINDOW     10
#define   r11_COMMUTATION_STEADY_PI     11
#define   r12_COMMUTATION_TRANSIENT_PI  12
#define   r13_BEMF_WINDOW         13
#define   r14_BEMF_FILTER         14
#define   r15_STARTUP_ALIGNMENT   15
#define   r16_STARTUP_MODE        16
#define   r17_WINDMILL_MODE       17
#define   r18_RAMP_FREQ           18
#define   r19_RAMP_DUTY           19
#define   r20_RAMP_STEP           20
#define   r21_SPEED_ACC           21
#define   r22_SPEED_RESOLUTION    22
#define   r23_SPEED_THRESHOLD     23
#define   r24_NVM_WRITE           24
#define   r25_SYSTEM_CONFIG       25
#define   r26_PHASE_ADVANCE       26
#define   r27_MOTORF_UNCTION      27
#define   r28_FAULT_MASK          28
#define   r29_READBACK_SELECT     29
#define   r30_DEMAND_INUPT        30
#define   r31_READBACK            31



//----------------------------------------------
#define ENABLED         1
#define DISABLED        0

//----------------------------------------------
// Register 0: PWM Configuration
//----------------------------------------------
//--Modulation Mode Select
#define MOD_PHASE3          0
#define MOD_PHASE2          1

//--Bridge PWM Mode Select
#define PMD_ALIGN_CENTER    0
#define PMD_ALIGN_EDGE      1

//----------------------------------------------
// Register 1: PWM Configuration
//----------------------------------------------
//--PWM Dither Step Period
#define DP_200NS    0
#define DP_400NS    1
#define DP_600NS    2
#define DP_800NS    3
#define DP_1000NS   4
#define DP_1200NS   5
#define DP_1400NS   6
#define DP_1600NS   7

//--PWM Dither Dwell Time
#define DD_1MS       0
#define DD_2MS       1
#define DD_5MS       2
#define DD_10MS      3

//----------------------------------------------
// Register 2: Bridge and Sense Amp Configuration
//----------------------------------------------
//--Sense Amp Gain
#define SA_2_5        0
#define SA_5          1
#define SA_10         2
#define SA_20         3

//----------------------------------------------
// Register 3: Gate Drive Configuration
// Register 4: Gate Drive Configuration
// Register 5: Gate Drive Configuration
//----------------------------------------------
//--Gate Register Select
#define CUR_TURN_OFF    0
#define CUR_TURN_ON     1


// Register 6: Current Limit Configuration

//----------------------------------------------
// Register 7: VDS Monitor and Sense Amp Configuration
//----------------------------------------------
//--Sense Amp Maximum Threshold
#define MIT_200MV    0
#define MIT_100MV    1
#define MIT_50MV     2
#define MIT_25MV     3

//----------------------------------------------
// Register 8: VDS Monitor Configuration
//----------------------------------------------
//--VDS Fault Qualifier Mode
#define VDQ_DEBOUNCE        0
#define VDQ_BLANK           1



// Register 9: Watchdog Configuration
// Register 10: Watchdog Configuration
// Register 11: Commutation Configuration
// Register 12: Commutation Configuration
// Register 13: BEMF Configuration

//----------------------------------------------
// Register 14: BEMF Configuration
//----------------------------------------------
//--BEMF Sampling
#define BS_SPC_1        0   // sample per cycle
#define BS_SPC_2        1
#define BS_SPC_3        2
#define BS_SPC_6        3

//--Windmill BEMF Filter Time
#define BF_0            0
#define BF_200US        1
#define BF_400US        2
#define BF_600US        3
#define BF_800US        4
#define BF_1MS          5
#define BF_2MS          6
#define BF_4MS          7
#define BF_5MS          8
#define BF_6MS          9
#define BF_10MS         10
#define BF_12MS         11
#define BF_14MS         12
#define BF_16MS         13
#define BF_18MS         14
#define BF_20MS         15
//----------------------------------------------
// Register 16: Startup Configuration
//----------------------------------------------
//--Start Coast Mode Select
#define STM_COAST_OFF       0
#define STM_COAST_ON        1

//--Restart Control after loss of sync
#define RSC_RESTART_OFF  0
#define RSC_RESTART_ON   1

//--Alignment Duty Cycle Ramp Time (%tHOLD)
#define HR_0P          0
#define HR_25P         1
#define HR_50P         2
#define HR_100P        3

//----------------------------------------------
// Register 17: Startup Configuration
//----------------------------------------------
#define WIN_DISABLED   0
#define WIN_ENABLED    1

// Register 18: Startup Configuration
// Register 19: Startup Configuration

//----------------------------------------------
// Register 20: Startup Configuration
//----------------------------------------------
//--Start Ramp Frequency Step
#define SFS_0_0125HZ    0
#define SFS_0_025HZ     1
#define SFS_0_05HZ      2
#define SFS_0_1HZ       3
#define SFS_0_2HZ       4
#define SFS_0_4HZ       5
#define SFS_0_8HZ       6
#define SFS_1HZ         7
#define SFS_1_5HZ       8
#define SFS_2HZ         9
#define SFS_2_5HZ       10
#define SFS_3HZ         11
#define SFS_5HZ         12
#define SFS_8HZ         13
#define SFS_10HZ        14
#define SFS_15HZ        15


// Register 21: Speed Control Loop Configuration

//----------------------------------------------
// Register 22: Speed Control Loop Configuration
//----------------------------------------------
//--Duty Cycle Compensation
#define DV_DISABLED    0
#define DV_12V         1
#define DV_24V         2
//#define DV_12V       3

//--Deceleration Factor
#define DF_1         0
#define DF_2         1
#define DF_5         2
#define DF_10        3

//--Speed Control Resolution
#define SR_0_1HZ       0
#define SR_0_2HZ       1
#define SR_0_4HZ       2
#define SR_0_8HZ       3
#define SR_1_6HZ       4
#define SR_3_2HZ       5
//#define SR_3_2HZ     6
//#define SR_3_2HZ     7


// Register 23: Speed Control Loop Configuration
// Register 24: Write NVM Control

//----------------------------------------------
// Register 25: System Configuration
//----------------------------------------------
//--Enable Stop On Fail Select
#define ESF_NOSTOP      0
#define ESF_STOP        1

//--Logic Regulator Voltage
#define VLR_3_3V        0
#define VLR_5V          1

//--Gate Drive Regulator Voltage
#define VRG_8V          0
#define VRG_11V         1

//--Operating Mode Select
#define OPM_SPI_ONLY     0
#define OPM_STAND_ALONE  1

//--Wake Mode Select
#define LWK_PWM         0
#define LWK_LIN         1

//--PWM Input Sense (when OPM = 1)
#define IPI_ACT_HIGH    0
#define IPI_ACT_LOW     1

//--Disable Current Limit (Speed Modes)
#define DIL_ENABLED      0
#define DIL_DISABLED     1

//--Selects Motor Control Mode
#define CM_CLOSE_LOOP_SPEED   0
//#define CM_CLOSE_LOOP_SPEED   1
#define CM_CLOSE_LOOP_CURRENT 2
#define CM_OPEN_LOOP          3

//----------------------------------------------
// Register 26: Phase Advance Select
//----------------------------------------------
//--Phase Advance Mode
#define PAM_MANUAL      0
#define PAM_AUTO        1

//--Auto Phase Advance Control Gain
#define KIP_GAIN_1      0
#define KIP_GAIN_2      1
#define KIP_GAIN_4      2
#define KIP_GAIN_8      3

//----------------------------------------------
// Register 27: Motor Function Control
//----------------------------------------------
//--Lin Enable
#define LIN_STANDBY     0
#define LIN_ACTIVE      1

//--Overmodulation Select
#define OVM_100P        0
#define OVM_112P        1
#define OVM_125P        2
#define OVM_150P        3

//--Drive Mode Select
#define DRM_SINUSOIDAL  0
#define DRM_TRAPEZOIDAL 1

//--Brake Function Select
#define BRK_ENABLED     0
#define BRK_DISABLE     1

//--Rotation Direction Select
#define DIR_FORWARD     0
#define DIR_REVERSE     1

//--Run enable
#define RUN_STOP        0
#define RUN_START       1

//----------------------------------------------
// Register 29: Readback Select
//----------------------------------------------
//--Selects output DIAG terminal
#define DGS_LOW_ON_FAULT    0
#define DGS_FG              1
#define DGS_PULSE_HIGH      2
#define DGS_PULSE_FG        3

//--Serial reset of fault state and fault bit
#define DSR_ENABLED     0
#define DSR_DISABLE     1

//--Selects LIN baud rate
#define LBR_10K         0
#define LBR_20K         1

//--Selects output on SDO whenSTRn = 1
#define CKS_HIGH_IMPEDANCE     0
#define CKS_DIV_SYSCLK         1

//--Selects data output on register 31
#define RBS_DIAGNOSTIC      0
#define RBS_MOTOR_SPEED     1
#define RBS_CURRENT         2
#define RBS_VOLTAGE         3
#define RBS_TEMPERATURE     4
#define RBS_DEMAND_INPUT    5
#define RBS_PEAK_DUTY_CYCLE 6
#define RBS_PHASE_ADVANCE   7


//==============================================================================
// Status Register
//==============================================================================
// FF  Status register flag (not including CLI)
// POR Power-on-reset
// SE  Serial transfer error
// VPU VPP undervoltage
// CLI Current limit*
// WD  Watchdog
// LOS Loss of bemf synchronization
// OT  Overtemperature
// TW  Temperature warning
// VSU VBB undervoltage
// VRU VREG undervoltage
// VLU VLR undervoltage
// BU  Bootstrap undervoltage
// VO  VDS fault
#define fbFF   (1<<15)
#define fbPOR  (1<<14)
#define fbSE   (1<<13)
#define fbVPU  (1<<12)
#define fbCLI  (1<<11)
//
#define fbWD   (1<<9)
#define fbLOS  (1<<8)
#define fbOT   (1<<7)
#define fbTW   (1<<6)
#define fbVSU  (1<<5)
#define fbVRU  (1<<4)
#define fbVLU  (1<<3)
#define fbBU   (1<<2)
#define fbVO   (1<<1)




extern U16 receiveData;


extern U8 configRUN (U8 run);
extern U8 A4964_Demand_Input (U16 DemandInput) ;
extern U8 A4964_ReadBack (void) ;
extern U32 A4964_ReadBack_Select (U8 rSel);




//==================================================================================================================
/*
//------------------------------------------------------------------------------------------------  PWM CONFIG

//==============================================================================
// PWMCONFIG [0] -- PWM Mode Setting
//==============================================================================
#define rPWM_CONFIG0         (0<<11)
//------------------------------------------------------------------------------
// MOD(b9), Selects 2-phase or 3-phase modulation.
#define MOD_3Phase          (0<<9)  // -- D(Default)
#define MOD_2Phase          (1<<9)
#define MOD                 MOD_3Phase
//------------------------------------------------------------------------------
// PMD(b7), Selects the bridge PWM mode.
#define PMD_Center_aligned  (0<<7)  // -- D
#define PMD_Edge_aligned    (1<<7)
#define PMD                 PMD_Center_aligned          
//------------------------------------------------------------------------------
// PW[5:0](b6-b1), a 6-bit integer to set the PWM period.
// tPW = 20.10 μs + (n × 0.8 μs) when PMD = 0   -- D(38)=19.8kHz
// tPW = 20.05 μs + (n × 0.8 μs) when PMD = 1
// n = 0~63   
#define tPW                 (38<<1)
//------------------------------------------------------------------------------ dPWM_CONFIG0

#define dPWM_CONFIG0  (rPWM_CONFIG0|MOD|PMD|tPW)


//==============================================================================
// PWMCONFIG [1] -- PWM Dither Setting
//==============================================================================
#define rPWM_CONFIG1         (1<<11)
//------------------------------------------------------------------------------
// DP[2:0](b9-b7) PWM Dither Step Period
// tPW = –0.2 μs – (n × 0.2 μs)
// tPW = –0.2 μs – (0 × 0.2 μs) -- D(0)=-0.2us
// n = 0~7 
#define DP                  (0<<7)
//------------------------------------------------------------------------------
// DD[1:0](b6-b5) PWM Dither Dwell Time
#define DD_1ms              (0<<5)  // D(0)
#define DD_2ms              (1<<5)
#define DD_5ms              (2<<5)
#define DD_10ms             (3<<5)
#define DD                  DD_1ms
//------------------------------------------------------------------------------
// DS[3:0](b4-b1) PWM Dither Step Count
// n = 0~ 15 -- D(0) Setting DS[3:0] to 0 will disable PWM dither.
#define DS                  (0<<1)

//------------------------------------------------------------------------------ dPWM_CONFIG1
#define dPWM_CONFIG1  (rPWM_CONFIG1|DP|DD|DS)                               



//==============================================================================
// BRIDGE_CONFIG [2] -- Bridge and Sense Amp Configuration
//==============================================================================
#define rBRIDGE_CONFIG       (2<<11)
//------------------------------------------------------------------------------
// SA[1:0](b8-b7) Sense Amp Gain
#define SA_Gain_2_5         (0<<7)  // 2.5 -- D
#define SA_Gain_5           (1<<7) 
#define SA_Gain_10          (2<<7) 
#define SA_Gain_20          (3<<7) 
#define SA                  SA_Gain_2_5
//------------------------------------------------------------------------------
// DT[5:0](b6-b1) Dead Time
// tDEAD = n × 50 ns
// n = 0~63 -- D(32)=1.6us
#define DT                  (32<<1)
//------------------------------------------------------------------------------ dBRIDGE_CONFIG

#define dBRIDGE_CONFIG  (rBRIDGE_CONFIG|SA|DT)





//------------------------------------------------------------------------------------------------  GATE DRIVER CONFIG

//==============================================================================
// GATEDRIVE0 [3] -- Gate Drive Configuration (Turn-On Current)
//==============================================================================
#define rGATE_DRIVE0        (3<<11)
//------------------------------------------------------------------------------
// IR1[3:0](b8-b5) Turn-On Current 1
// IR1 = n × –5 mA
// n=0~15 -- D(0)=0  setting 0 turn on the MOSFET as quickly as possibl
#define IR1                 (0<<5)
//------------------------------------------------------------------------------
// IR2[3:0](b4-b1) Turn-On Current 2
// IR2 = n × –5 mA
// n=0~15 -- D(0)=0  setting 0 turn on the MOSFET as quickly as possibl
#define IR2                 (0<<1)

//------------------------------------------------------------------------------ dGATE_DRIVE0

#define dGATE_DRIVE0  (rGATE_DRIVE0|IR1|IR2)

//==============================================================================
// GATEDRIVE1 [4] -- Gate Drive Configuration (Turn-Off Current)
//==============================================================================
#define rGATE_DRIVE1        (4<<11)
//------------------------------------------------------------------------------
// IF1[3:0](b8-b5) Turn-Off Current 1
// IF1 = n × –5 mA
// n=0~15 -- D(0)=0  setting 0 turn on the MOSFET as quickly as possibl
#define IF1                 (0<<5)
//------------------------------------------------------------------------------
// IF2[3:0](b4-b1) Turn-Off Current 2
// IF2 = n × –5 mA
// n=0~15 -- D(0)=0  setting 0 turn on the MOSFET as quickly as possibl
#define IF2                 (0<<1)

//------------------------------------------------------------------------------ dGATE_DRIVE1

#define dGATE_DRIVE1  (rGATE_DRIVE1|IF1|IF2)


//==============================================================================
// GATEDRIVE2 [5] -- Gate Drive Configuration (Slew Control Turn-On/Off Time)
//==============================================================================
#define rGATE_DRIVE2        (5<<11)
//------------------------------------------------------------------------------
// TRS[3:0](b8-b5) Slew Control Turn-On Time
// tRS = n × 50 ns
// n=0~15 -- D(0)=0  The range of tRS is 0 ns to 750 ns
#define TRS                 (0<<5)
//------------------------------------------------------------------------------
// TFS[3:0](b4-b1) Slew Control Turn-Off Time
// tFS = n × 50 ns
// n=0~15 -- D(0)=0  The range of tRS is 0 ns to 750 ns
#define TFS                 (0<<1)

//------------------------------------------------------------------------------ dGATE_DRIVE2

#define dGATE_DRIVE2  (rGATE_DRIVE2|TRS|TFS)



//------------------------------------------------------------------------------------------------  Current Limit Configuration

//==============================================================================
// CURRENT_LIMIT [6] -- Current Limit Configuration
//==============================================================================
#define rCURRENT_LIMIT      (6<<11)
//------------------------------------------------------------------------------
// OBT[4:0](b9~b5) Current Limit Blank Time
// tOCB = (n + 2) × 200 ns
// n = 0~31 -- D(7)=1.8us ,The range of tOCB is 1 us to 6.6 us.
#define OBT                 (7<<5)
//------------------------------------------------------------------------------
// VIL[3:0](b4-b1) Current Limit Scale
// n = 0~15
//---------------------------
// n : scale 
//---------------------------
// 0 : 1/16 |  8 : 9/16
// 1 : 2/16 |  9 : 10/16
// 2 : 3/16 | 10 : 11/16
// 3 : 4/16 | 11 : 12/16
// 4 : 5/16 | 12 : 13/16
// 5 : 6/16 | 13 : 14/16
// 6 : 7/16 | 14 : 15/16
// 7 : 8/16 | 15 : 16/16 -- D(15)Scale=1,  VILIM defined by Scale × Maximum Threshold (MIT[1:0])
//---------------------------
#define VIL                 (15<<1)

//------------------------------------------------------------------------------ dCURRENT_LIMIT

#define dCURRENT_LIMIT  (rCURRENT_LIMIT|OBT|VIL)




//------------------------------------------------------------------------------------------------  VDS Monitor and Sense Amp Configuration

//==============================================================================
//  VDS_MONITOR0 [7] -- VDS Monitor and Sense Amp Configuration
//==============================================================================
#define rVDS_MONITOR0       (7<<11)
//------------------------------------------------------------------------------
// MIT[1:0](b9-b8) Sense Amp Maximum Threshold
#define MIT_200mv           (0<<8)  // D(0)
#define MIT_100mv           (1<<8) 
#define MIT_50mv            (2<<8) 
#define MIT_25mv            (3<<8) 
#define MIT                 MIT_200mv
//------------------------------------------------------------------------------
// VT[5:0](b6-b1) VDS Overvoltage Threshold
// VDST = n × 50 mV -- D(31)=1.55v , The range of VDST is 0 to 3.15 V.
#define VT                  (31<<1)

//------------------------------------------------------------------------------ dVDS_MONITOR0

#define dVDS_MONITOR0  (rVDS_MONITOR0|MIT|VT)


//==============================================================================
//  VDS_MONITOR1 [8] -- VDS Monitor and Sense Amp Configuration
//==============================================================================
#define rVDS_MONITOR1       (8<<11)
//------------------------------------------------------------------------------
// VDQ(b8) VDS Fault Qualifier Mode
#define VDQ_Debounce        (0<<8)  // D(0)
#define VDQ_Blank           (1<<8)
#define VDQ                 VDQ_Debounce
//------------------------------------------------------------------------------
// VQT[5:0](b6-b1) VDS Qualify Time
// tVDQ = n × 50 ns -- D(63)=3.15us ,The useable range of tVDQ is 600 ns to 3.15 ?s
// n = 0~63
#define VQT                (63<<1)

//------------------------------------------------------------------------------ dVDS_MONITOR1

#define dVDS_MONITOR1  (rVDS_MONITOR1|VDQ|VQT)



//------------------------------------------------------------------------------------------------  Watchdog Configuration
//==============================================================================
//  WATCHDOG_CONFIG0 [9] -- Watchdog Minimum Time
//==============================================================================
#define rWATCHDOG_CONFIG0   (9<<11)
//------------------------------------------------------------------------------
// WM[4:0](b5-b1) Watchdog Minimum Time
// tWM = 1 + (n × 2) ms -- D(0)=1ms ,The range of tWM is 1 ms to 63 ms.
// n = 0~31
#define WM                  (0<<1)
//------------------------------------------------------------------------------ dWATCHDOG_CONFIG0

#define dWATCHDOG_CONFIG0   (rWATCHDOG_CONFIG0|WM)


//==============================================================================
//  WATCHDOG_CONFIG1 [10] -- Watchdog Window Time & Fail Cycle Count
//==============================================================================
#define rWATCHDOG_CONFIG1   (10<<11)
//------------------------------------------------------------------------------
// WC[3:0](b9-b6) Watchdog Fail Cycle Count Before Sleep
// n = 0 ~ 15 -- D(0) , WC[3:0] to [0000] disables the watchdog cycle counter
#define WC                  (0<<6)
//------------------------------------------------------------------------------
// WW[4:0](b5-b1) Watchdog Window Time
// tWW = 10 + (n × 10) ms -- D(0)=10ms ,TThe range of tWW is 10 ms to 320 ms
// n = 0~31
#define WW                  (0<<1)
//------------------------------------------------------------------------------ dWATCHDOG_CONFIG1

#define dWATCHDOG_CONFIG1  (rWATCHDOG_CONFIG1|WC|WW)



//------------------------------------------------------------------------------------------------  Commutation Configuration
//==============================================================================
//  COMMUTATION_CONFIG0 [11] -- Steady-State Commutation Controlle P,I
//==============================================================================
#define rCOMMUTATION_CONFIG0       (11<<11)
//------------------------------------------------------------------------------
// CP[3:0](b8-b5) Steady-State Commutation Controller Proportional Gain
// Position control proportional gain is KCP
// KCP = 2^(n-7) -- D(7)=1 , The range of KCP is 1/128 to 256
// n = 0~15
#define CP                  (7<<5)
//------------------------------------------------------------------------------
// CI[3:0](b4-b1) Steady-State Commutation Controller Integral Gain
// Position control proportional gain is KCI
// KCI = 2^(n-7) -- D(7)=1 , The range of KCI is 1/128 to 256
#define CI                  (7<<1)
//------------------------------------------------------------------------------ rCOMMUTATION_CONFIG0

#define dCOMMUTATION_CONFIG0  (rCOMMUTATION_CONFIG0|CP|CI)



//==============================================================================
//  COMMUTATION_CONFIG1 [12] -- Transient Commutation Controller P,I
//==============================================================================
#define rCOMMUTATION_CONFIG1       (12<<11)
//------------------------------------------------------------------------------
// CPT[3:0](b8-b5) Transient Commutation Controller Proportional Gain
// Position control proportional gain is KCPT
// KCPT = 2^(n-7) -- D(0)=1/128 , The range of KCPT is 1/128 to 256
// n = 0~15
#define CPT                 (0<<5)
//------------------------------------------------------------------------------
// CIT[3:0](b4-b1) Transient Commutation Controller Integral Gain
// Position control proportional gain is KCIT
// KCIT = 2^(n-7) -- D(0)=1/128 , The range of KCIT is 1/128 to 256
#define CIT                 (0<<1)
//------------------------------------------------------------------------------ dCOMMUTATION_CONFIG1

#define dCOMMUTATION_CONFIG1  (rCOMMUTATION_CONFIG1|CPT|CIT)


//------------------------------------------------------------------------------------------------  BEMF Configuration

//==============================================================================
//  BEMF_CONFIG0 [13] -- BEMF Detection Window
//==============================================================================
#define rBEMF_CONFIG0       (13<<11)
//------------------------------------------------------------------------------
// BW[4:0](b5-b1) BEMF Detection Window
// Position control proportional gain is KCP
// θBW = (n + 1) × 1.4°(elec) |n=0..30
// θBW = 60°(elec)            |n=31
// n = 0~31 -- D(4)=7°(elec) , The range of θBW is 1.4° to 43.4° and 60°
#define BW                 (4<<1)
//------------------------------------------------------------------------------ dBEMF_CONFIG0

#define dBEMF_CONFIG0  (rBEMF_CONFIG0|BW)



//==============================================================================
//  BEMF_CONFIG1 [14] -- BEMF Sampling & Filter Time
//==============================================================================
#define rBEMF_CONFIG1       (14<<11)
//------------------------------------------------------------------------------
// BS[1:0](b6-b5) BEMF Sampling (Samples per Cycle)
#define BS_1                (0<<5)  // D(0)
#define BS_2                (1<<5)
#define BS_3                (2<<5)
#define BS_6                (3<<5)
#define BS                  BS_1
//------------------------------------------------------------------------------
// BF[3:0](b4-b1) Windmill BEMF Filter Time
// n = 0~15
//---------------------------
// n : scale 
//---------------------------
// 0 :   0us |  8 : 5ms
// 1 : 200us |  9 : 6ms
// 2 : 400us | 10 : 10ms
// 3 : 600us | 11 : 12ms
// 4 : 800us | 12 : 14ms
// 5 : 1ms   | 13 : 15ms
// 6 : 2ms   | 14 : 16ms
// 7 : 4ms   | 15 : 20ms -- D(1)=200us
//---------------------------
#define BF                  (1<<1)
//------------------------------------------------------------------------------ dBEMF_CONFIG1

#define dBEMF_CONFIG1  (rBEMF_CONFIG1|BS|BF)




//------------------------------------------------------------------------------------------------  Startup Configuration


//==============================================================================
//  STARTUP_CONFIG0 [15] -- Alignment (Hold) Time / Peak PWM Duty During Alignment
//==============================================================================
#define rSTARTUP_CONFIG0           (15<<11)
//------------------------------------------------------------------------------
// HT[3:0](b9-b6) Alignment (Hold) Time
// tHOLD = n × 200 ms
// n = 0~15 -- D(1)=200 ms, The range of tHOLD is 0 to 3 s
#define HT                  (1<<6)
//------------------------------------------------------------------------------
// HD[4:0](b5-b1) Peak PWM Duty During Alignment
// DH = (n + 1) × 3.125%
// n = 1~31 -- D(5)=18.75% , The range of DH is 3.125% to 100%.
#define DH                  (5<<1)

//------------------------------------------------------------------------------ dSTARTUP_CONFIG0 (Alignment)

#define dSTARTUP_CONFIG0  (rSTARTUP_CONFIG0|HT|DH)



//==============================================================================
//  STARTUP_CONFIG1 [16] -- Coast Mode / Restart / Motor Constant / Alignment Duty Cycle Ramp Time 
//==============================================================================
#define rSTARTUP_CONFIG1           (16<<11)
//------------------------------------------------------------------------------
// STM(b9) Start Coast Mode Select
#define STM_COAST_DISABLED  (0<<9)  // D(0)
#define STM_COAST_ENABLED   (1<<9) 
#define STM                 STM_COAST_DISABLED
//------------------------------------------------------------------------------
// RSC(b8) Restart Control
#define RSC_NO_RESTART      (0<<8)  // D(0)
#define RSC_RESTART         (1<<8) 
#define RSC                 RSC_NO_RESTART
//------------------------------------------------------------------------------
// KM[3:0](b6-b3) Motor Constant (Ratio Between Speed and BEMF)
// KM = 0.3 + (n × 0.05)
// n = 0~15 -- D(7)=0.65 , The range of KM is 0.3 to 1.05.
#define KM                  (7<<3)
//------------------------------------------------------------------------------
// HR[1:0](b2-b1) Alignment Duty Cycle Ramp Time
#define HR_0                (0<<1)  // D(0)
#define HR_25               (1<<1)  // 25% tHOLD Time to reach peak DH
#define HR_50               (2<<1)  // 50%
#define HR_100              (3<<1)  // 100%
#define HR                  HR_0

//------------------------------------------------------------------------------ dSTARTUP_CONFIG1 (Alignment)

#define dSTARTUP_CONFIG1  (rSTARTUP_CONFIG1|STM|RSC|KM|HR)




//==============================================================================
//  STARTUP_CONFIG2 [17] -- Windmill
//==============================================================================
#define rSTARTUP_CONFIG2           (17<<11)
//------------------------------------------------------------------------------
// WIN(b8) Windmill Mode Select
#define WINDMILL_DISABLED   (0<<8)  // D
#define WINDMILL_ENABLED    (1<<8)
#define WIN                 WINDMILL_DISABLED
//------------------------------------------------------------------------------
// WMF[2:0](b7-b5) Minimum Windmill Detection Frequency
// fWM = 0.4 + (n × 3.2) Hz
// n = 0~7 -- D(2)=6.8Hz
#define WMF                 (2<<5)
//------------------------------------------------------------------------------
// WBD[3:0](b4-b1) Duty Cycle During Windmill Braking
// DWB = (n + 1) × 6.25%
// n = 0~15 -- D(7)=50% , The range of DWB is 6.25% to 100%.
#define WBD                 (7<<1)
//------------------------------------------------------------------------------ dSTARTUP_CONFIG2 (WindMill)

#define dSTARTUP_CONFIG2  (rSTARTUP_CONFIG2|WIN|WMF|WBD)


//==============================================================================
//  STARTUP_CONFIG3 [18] -- Start Ramp Initial/Final Frequency
//==============================================================================
#define rSTARTUP_CONFIG3           (18<<11)
//------------------------------------------------------------------------------
// SF2[3:0](b8-b5) Start Ramp Final Frequency
// fS2 = 10 + (n × 2.5) Hz
// n = 0~15 -- D(7)=27.5Hz , The range of fS2 is 10 Hz to 47.5 Hz.
#define SF2                 (7<<5)
//------------------------------------------------------------------------------
// SF1[3:0](b4-b1) Start Ramp Initial Frequency
// fS1 = 0.5 + (n × 0.5) Hz
// n = 0~15 -- D(7)=4Hz , The range of fS1 is 0.5 Hz to 8 Hz.
#define SF1                 (7<<1)
//------------------------------------------------------------------------------ dSTARTUP_CONFIG3 (Start Ramp Frequency)

#define dSTARTUP_CONFIG3  (rSTARTUP_CONFIG3|SF2|SF1)

        
//==============================================================================
//  STARTUP_CONFIG_CONFIG4 [19] -- Start Ramp Initial/Final Duty Cycle
//==============================================================================
#define rSTARTUP_CONFIG4           (19<<11)
//------------------------------------------------------------------------------
// SD2[3:0](b8-b5) Start Ramp Final Duty Cycle
// DS2 = (n + 1) × 6.25%
// n = 0~15 -- D(7)=31.25% , The range of DS2 is 6.25% to 100%
#define SD2                 (7<<5)
//------------------------------------------------------------------------------
// SD1[3:0](b4-b1) Start Ramp Initial Duty Cycle
// DS1 = (n + 1) × 6.25%
// n = 0~15 -- D(7)=50% , The range of DS1 is 6.25% to 100%
#define SD1                 (7<<1)
//------------------------------------------------------------------------------ dSTARTUP_CONFIG4 (Start Ramp Duty)

#define dSTARTUP_CONFIG4  (rSTARTUP_CONFIG4|SD2|SD1)




//==============================================================================
//  STARTUP_CONFIG_CONFIG5 [20] -- Start Ramp Step Time / Start Ramp Frequency Step
//==============================================================================
#define rSTARTUP_CONFIG5           (20<<11)
//------------------------------------------------------------------------------
// STS[3:0](b8-b5) Start Ramp Step Time
// tSS = 10 ms (n=0)
// tSS = n × 20 ms (n=1..15)
// n = 0~15 -- D(4)=80ms , The range of tSTS is 10 ms to 300 ms.
#define STS                 (4<<5)
//------------------------------------------------------------------------------
// SFS[3:0](b4-b1) Start Ramp Frequency Step
//---------------------------
// Frequency Step
//---------------------------
// 0 : 0.0125 Hz |  8 : 1.5 Hz
// 1 : 0.025 Hz  |  9 : 2 Hz
// 2 : 0.05 Hz   | 10 : 2.5 Hz
// 3 : 0.1 Hz    | 11 : 3 Hz
// 4 : 0.2 Hz    | 12 : 5 Hz
// 5 : 0.4 Hz    | 13 : 8 Hz
// 6 : 0.8 Hz    | 14 : 10 Hz
// 7 : 1 Hz      | 15 : 15 Hz -- D(7)=1Hz
//---------------------------
#define SFS                 (7<<1)
//------------------------------------------------------------------------------ rSTARTUP_CONFIG5 (Start Ramp Step Time)

#define dSTARTUP_CONFIG5  (rSTARTUP_CONFIG5|STS|SFS)




//------------------------------------------------------------------------------------------------  Speed Control Loop Configuration

//==============================================================================
//  SPEEDLOOP_CONFIG0 [21] --  Speed Control Acceleration Limit / Speed Control Gain
//==============================================================================
#define rSPEEDLOOP_CONFIG0         (21<<11)
//------------------------------------------------------------------------------
// SGL[4:0](b9-b5) Speed Control Acceleration Limit
// KSL = 6.3 + (n × 6.4) Hz
// n = 0~31 -- D(5)=38.3Hz , The range of KSL is 6.3 Hz to 204.7 Hz.
#define SGL                 (5<<5)
//------------------------------------------------------------------------------
// SG[3:0](b4-b1) Speed Control Gain
// KS = 1 + (n × 2)
// n = 0~15 -- D(5)=11 , The range of KS is 1 to 31
#define SG                  (5<<1)

//------------------------------------------------------------------------------ dSPEEDLOOP_CONFIG0 (Speed Control Gain)

#define dSPEEDLOOP_CONFIG0  (rSPEEDLOOP_CONFIG0|SGL|SG)



//==============================================================================
//  SPEEDLOOP_CONFIG1 [22] --  Duty Cycle Compensation / Speed Control Resolution
//==============================================================================
#define rSPEEDLOOP_CONFIG1         (22<<11)
//------------------------------------------------------------------------------
// DV[1:0](b9-b8) Duty Cycle Compensation
#define DV_DISABLED         (0<<8)
#define DV_12V              (1<<8)  // -- D
#define DV_24V              (2<<8)
//#define DV_12V            (3<<8)
#define DV                  DV_12V
//------------------------------------------------------------------------------
// DF[1:0](b7-b6) Deceleration Factor
#define DF_1                (0<<6)  // -- D
#define DF_2                (1<<6)
#define DF_5                (2<<6)
#define DF_10               (3<<6)
#define DF                  DF_1
//------------------------------------------------------------------------------
// SR[2:0](b3-b1) Speed Control Resolution
#define SR_0_1              (0<<1)  // 0.1Hz -- D 
#define SR_0_2              (1<<1)
#define SR_0_4              (2<<1)
#define SR_0_8              (3<<1)
#define SR_1_6              (4<<1)
#define SR_3_2              (5<<1)
//#define SR_3_2            (6<<1)
//#define SR_3_2            (7<<1)
#define SR                  SR_0_1

//------------------------------------------------------------------------------ dSPEEDLOOP_CONFIG1 (Speed Control Resolution)

#define dSPEEDLOOP_CONFIG1  (rSPEEDLOOP_CONFIG1|DV|DF|SR)


//==============================================================================
//  SPEEDLOOP_CONFIG2 [23] --  Underspeed (High/Low-Speed) Threshold
//==============================================================================
#define rSPEEDLOOP_CONFIG2         (23<<11)
//------------------------------------------------------------------------------
// SL[3:0](b8-b5) Underspeed (Low-Speed) Threshold
// fSL = 8 × n × fRES (Hz) (fRES=SR[2:0] Speed Control Resolution)
// n = 0~15 -- D(7)=8*7*0.1=5.6Hz
#define SL                  (7<<5)
//------------------------------------------------------------------------------
// SH[3:0](b4-b1) Underspeed (High-Speed) Threshold
// fSH = [127 + (n × 128)] × fRES (Hz) (fRES=SR[2:0] Speed Control Resolution)
// n = 0~15 -- D(7)=(127+7*128)*0.1=102.3Hz
#define SH                  (7<<1)

//------------------------------------------------------------------------------ dSPEEDLOOP_CONFIG2 (Underspeed (High/Low-Speed) Threshold)

#define dSPEEDLOOP_CONFIG2  (rSPEEDLOOP_CONFIG2|SL|SH)



//------------------------------------------------------------------------------------------------  Write NVM Control
//==============================================================================
//  NVMWRITE [24] --  Write NVM Control
//==============================================================================
#define rNVM_WRITE          (24<<11)
//------------------------------------------------------------------------------
// SAV[1:0](b9-b8) Save Parameters to Non-Volatile Memory (NVM)
// When SAV[1:0] is changed from 01 to 10,register will be written to NVM.
// When the NVM save has completed successfully, SAV[1:0] will be set to 01
// If SAV[1:0] is reset to 00, the save has not completed successfully
#define WR_NVM_INIT         (0<<8)  // D
#define WR_NVM_BUSY         (0<<8)  // when writting check status
#define WR_NVM_FINISH       (1<<8)
#define WR_NVM_START        (2<<8)

#define dNVM_WRITE  (rNVM_WRITE|WR_NVM_INIT)



//------------------------------------------------------------------------------------------------  System Configuration
//==============================================================================
//  SYSTEM_CONFIG [25] --  System Configuration
//==============================================================================
#define rSYSTEM_CONFIG          (25<<11)
//------------------------------------------------------------------------------
// ESF(b9) Enable Stop On Fail Select
#define ESF_NO_STOP_ON_FAIL     (0<<9)
#define ESF_STOP_ON_FAIL        (1<<9)  // -- D
#define ESF                     ESF_STOP_ON_FAIL
//------------------------------------------------------------------------------
// VLR(b8) Logic Regulator Voltage
#define VLR_3_3V                (0<<8)  // -- D
#define VLR_5V                  (1<<8)
#define VLR                     VLR_3_3V
//------------------------------------------------------------------------------
// VRG(b7) Gate Drive Regulator Voltage
#define VRG_8V                  (0<<7)
#define VRG_11V                 (1<<7)  // -- D
#define VRG                     VRG_11V
//------------------------------------------------------------------------------
// OPM(b6) Operating Mode Select
#define OPM_SPI_ONLY            (0<<6)  // -- D
#define OPM_STANDBY_ALONE       (1<<6)
#define OPM                     OPM_SPI_ONLY
//------------------------------------------------------------------------------
// LWK(b5) Wake Mode Select
#define LWK_PWM_WAKE            (0<<5)  // -- D
#define LWK_LIN_WAKE            (1<<5)
#define LWK                     LWK_PWM_WAKE
//------------------------------------------------------------------------------
// IPI(b4) PWM Input Sense (when OPM = 1)
#define IPI_ACTIVE_HIGH         (0<<4)  // -- D
#define IPI_ACTIVE_LOW          (1<<4)
#define IPI                     IPI_ACTIVE_HIGH
//------------------------------------------------------------------------------
// DIL(b3) Disable Current Limit (Speed Modes)
#define DIL_EN                  (0<<3)  // -- D
#define DIL_DIS                 (1<<3)
#define DIL                     DIL_EN
//------------------------------------------------------------------------------
// CM[1:0](b2-b1) Selects Motor Control Mode
#define CM_CLOSE_LOOP1          (0<<1)  // -- D
#define CM_CLOSE_LOOP2          (1<<1)
#define CM_CLOSE_LOOP_CURRENT   (2<<1)
#define CM_OPEN_LOOP            (3<<1)
#define CM                      CM_CLOSE_LOOP1

//------------------------------------------------------------------------------ dSYSTEM_CONFIG

#define dSYSTEM_CONFIG  (rSYSTEM_CONFIG|ESF|VLR|VRG|OPM|LWK|IPI|DIL|CM)
                                        


//------------------------------------------------------------------------------------------------   Phase Advance Select
//==============================================================================
//  PHASE_ADVANCE [26] --  Phase Advance Select
//==============================================================================
#define rPHASE_ADVANCE          (26<<11)
//------------------------------------------------------------------------------
// PAM(b9) Phase Advance Mode
#define PAM_MANUAL              (0<<9)
#define PAM_AUTOMATIC           (1<<9)
#define PAM                     PAM_MANUAL
//------------------------------------------------------------------------------
// KIP[1:0](b8-b7) Auto Phase Advance Control Gain
#define KIP_1                   (0<<8)  // D
#define KIP_2                   (1<<8)
#define KIP_4                   (2<<8)
#define KIP_8                   (3<<8)
#define KIP                     KIP_1
//------------------------------------------------------------------------------
// PA[5:0](b6-b1) Phase Advance
// θADV = n × 0.7°(elec) (n=0..62)
// θADV = 60°(elec) (n=63)
// n = 0~63 -- D(0)=0
#define PA                      (0<<1)

//------------------------------------------------------------------------------ dPHASE_ADVANCE

#define dPHASE_ADVANCE  (rPHASE_ADVANCE|PAM|KIP|PA)




//------------------------------------------------------------------------------------------------   Motor Function Control
//==============================================================================
//  MOTOR_FUNCTION [27] --  Motor Function Control
//==============================================================================
#define rMOTOR_FUNCTION         (27<<11)
//------------------------------------------------------------------------------
// LEN(b8) Lin Enable)
// 0: STANDBY
// 1: ACTIVE
#define LEN_STANDBY             (0<<8)
#define LEN_ACTIVE              (1<<8)
#define LEN                     LEN_STANDBY
//------------------------------------------------------------------------------
// GTS(b7) Go to Sleep Command
//-----------------------------
//GTS  Sleep transition
//0    No change in state
//1    No change in state
//1→0 No change in state
//0→1 Enter sleep state if enabled
#define GTS_0                   (0<<7)
#define GTS_1                   (1<<7)
#define GTS                     GTS_0
//------------------------------------------------------------------------------
// OVM[1:0](b6-b5) Overmodulation Select
// 0: 100%
// 1: 112.5%
// 2: 125%
// 3: 150%
#define OVM_100                 (0<<5)  // D
#define OVM_112                 (1<<5)
#define OVM_125                 (2<<5)
#define OVM_150                 (3<<5)
#define OVM                     OVM_100
//------------------------------------------------------------------------------
// DRM(b4) Drive Mode Select
// 0: Sinusoidal
// 1: Trapezoidal
#define DRM_Sinusoidal          (0<<4)  // D
#define DRM_Trapezoidal         (1<<4)
#define DRM                     DRM_Sinusoidal
//------------------------------------------------------------------------------
// BRK(b3) Brake Function Select
// 0: DISABLE
// 1: ENABLE
#define BRK_Disabled            (0<<3)  // D
#define BRK_Enabled             (1<<3)
#define BRK                     BRK_Disabled
//------------------------------------------------------------------------------
// DIR(b2) Rotation Direction Select
// 0: Forward
// 1: Reverse
#define DIR_Forward             (0<<2)  // D
#define DIR_Reverse             (1<<2)
#define DIR                     DIR_Forward
//------------------------------------------------------------------------------
// RUN(b1) Run enable
// 0: STOP
// 1: RUN
#define RUN_STOP                (0<<1)  // D
#define RUN_START               (1<<1)
#define RUN                     RUN_STOP

//------------------------------------------------------------------------------ dMOTOR_FUNCTION

#define dMOTOR_FUNCTION  (rMOTOR_FUNCTION|LEN|GTS|OVM|DRM|BRK|DIR|RUN)




//------------------------------------------------------------------------------------------------  FAULT MASK
//==============================================================================
//  FAULT_MASK [28] -- Mask Registe
//==============================================================================
#define rFAULT_MASK         (28<<11)
//------------------------------------------------------------------------------
// WD(b9) Watchdog
// 0: fault enable
// 1: fault mask
#define WD_FAULT_EN         (0<<9)
#define WD_MASK             (1<<9)
#define WD                  WD_MASK
//------------------------------------------------------------------------------
// LOS(b8) Loss of bemf synchronization
#define LOS_FAULT_EN        (0<<8)
#define LOS_MASK            (1<<8)
#define LOS                 LOS_FAULT_EN
//------------------------------------------------------------------------------
// OT(b7) Overtemperature
#define OT_FAULT_EN         (0<<7)
#define OT_MASK             (1<<7)
#define OT                  OT_FAULT_EN
//------------------------------------------------------------------------------
// TW(b6) Temperature warning
#define TW_FAULT_EN         (0<<6)
#define TW_MASK             (1<<6)
#define TW                  TW_FAULT_EN
//------------------------------------------------------------------------------
// VSU(b5) VBB undervoltage
#define VSU_FAULT_EN        (0<<5)
#define VSU_MASK            (1<<5)
#define VSU                 VSU_FAULT_EN
//------------------------------------------------------------------------------
// VRU(b4) VREG undervoltage
#define VRU_FAULT_EN        (0<<4)
#define VRU_MASK            (1<<4)
#define VRU                 VRU_FAULT_EN
//------------------------------------------------------------------------------
// VLU(b3) VLR undervoltage
#define VLU_FAULT_EN        (0<<3)
#define VLU_MASK            (1<<3)
#define VLU                 VLU_FAULT_EN
//------------------------------------------------------------------------------
// BU(b2) Bootstrap undervoltage
#define BU_FAULT_EN         (0<<2)
#define BU_MASK             (1<<2)
#define BU                  BU_FAULT_EN
//------------------------------------------------------------------------------
// VO(b1) VDS overvoltage
#define VO_FAULT_EN         (0<<1)
#define VO_MASK             (1<<1)
#define VO                  VO_FAULT_EN
//------------------------------------------------------------------------------ dMOTOR_FUNCTION

#define dFAULT_MASK (rFAULT_MASK |WD|LOS|OT|TW|VSU|VRU|VLU|BU|VO)



//------------------------------------------------------------------------------------------------  Readback Select
//==============================================================================
//  READBACK [29] -- Readback Select
//==============================================================================
#define rREADBACK_SELECT           (29<<11)
//------------------------------------------------------------------------------
// DGS[1:0](b9-b8) Selects output DIAG terminal.
// 0: Active low fault flag -- D
// 1: FG; high when motor is stationary
// 2: Pulse output; high when no fault present
// 3: Pulse output; FG when no fault present
#define DGS                 (0<<8)
//------------------------------------------------------------------------------
// DSR(b7) Serial reset of fault state and fault bit
// Reset on serial read
// 0: Enable -- D
// 1: Disable
#define DSR_Enabled         (0<<7)  // D
#define DSR_Disabled        (1<<7)
#define DSR                 DSR_Enabled
//------------------------------------------------------------------------------
// LBR(b5) Selects LIN baud rate
// 0: 10kHz -- D
// 1: 20kHz
#define LBR_10K             (0<<5)  // D
#define LBR_20K             (1<<5)
#define LBR                 LBR_10K
//------------------------------------------------------------------------------
// CKS(b4) Selects output on SDO when STRn = 1
// 0: High impedance -- D
// 1: Divided system clock
#define CKS                 (0<<4)
//------------------------------------------------------------------------------
// RBS[2:0](b3-b1) Selects data output on register 31
// Register 31 contents -- D
// 0: Diagnostic registe
// 1: Motor speed
// 2: Average supply current
// 3: Supply voltage
// 4: Chip temperature
// 5: Demand input
// 6: Applied bridge peak duty cycle
// 7: Applied phase advance
#define RBS                 (0<<1)

//------------------------------------------------------------------------------ dREADBACK_SELECT

#define dREADBACK_SELECT (rREADBACK_SELECT |DGS|DSR|LBR|CKS|RBS)



//------------------------------------------------------------------------------------------------  Demand input
//==============================================================================
//  DEMAND_INUPT [30] -- Write Only
//==============================================================================
#define rDEMAND_INUPT       (30<<11)
//------------------------------------------------------------------------------
// DI[9]~DI[0] 10bits 
// n = 0 ~ 1023
#define DEMAND_DATA         0

//------------------------------------------------------------------------------ dDEMAND_INUPT

#define dDEMAND_INUPT       (rDEMAND_INUPT|DEMAND_DATA)




const uint16_t A4964_CONFIG[] =
{
    dPWM_CONFIG0,       // CONFIG[0]
    dPWM_CONFIG1,       // CONFIG[1]
    dBRIDGE_CONFIG,     // CONFIG[2]
    dGATE_DRIVE0,       // CONFIG[3]
    dGATE_DRIVE1,       // CONFIG[4]
    dGATE_DRIVE2,       // CONFIG[5]
    dCURRENT_LIMIT,     // CONFIG[6]
    dVDS_MONITOR0,      // CONFIG[7]
    dVDS_MONITOR1,      // CONFIG[8]
    dWATCHDOG_CONFIG0,         // CONFIG[9]
    dWATCHDOG_CONFIG1,         // CONFIG[10]
    dCOMMUTATION_CONFIG0,      // CONFIG[11]
    dCOMMUTATION_CONFIG1,      // CONFIG[12]
    dBEMF_CONFIG0,      // CONFIG[13]
    dBEMF_CONFIG1,      // CONFIG[14]
    dSTARTUP_CONFIG0,          // CONFIG[15]
    dSTARTUP_CONFIG1,          // CONFIG[16]
    dSTARTUP_CONFIG2,          // CONFIG[17]
    dSTARTUP_CONFIG3,          // CONFIG[18]
    dSTARTUP_CONFIG4,          // CONFIG[19]
    dSTARTUP_CONFIG5,          // CONFIG[20]
    dSPEEDLOOP_CONFIG0,        // CONFIG[21]
    dSPEEDLOOP_CONFIG1,        // CONFIG[22]
    dSPEEDLOOP_CONFIG2,        // CONFIG[23]
    dNVM_WRITE,          // CONFIG[24]
    dSYSTEM_CONFIG,     // CONFIG[25]
    dPHASE_ADVANCE,     // CONFIG[26]
    dMOTOR_FUNCTION,    // CONFIG[27]
    dFAULT_MASK,        // CONFIG[28]
    dREADBACK_SELECT,          // CONFIG[29]
    dDEMAND_INUPT,      // CONFIG[30]
};
*/



//==================================================================================================================
/*

//==============================================================================
#define A4964_Writerbit     0x0400

//==============================================================================
#define PWM_config          0x0000
//==============================================================================
// MOD, Selects 2-phase or 3-phase modulation.
#define MOD_3_Phase         0x0000
#define MOD_2_Phase         0x0200
//==========================================================
// PMD, Selects the bridge PWM mode.
#define PMD_Center_aligned  0x0000
#define PMD_Edge_aligned    0x0080
//==========================================================
// PW[5 0], a 6-bit integer to set the PWM period.


//==============================================================================
#define PWM_dither          0x0800
//==============================================================================
// DP[2 0], 3 bits to select the dither step period.
//==========================================================
// DD[1 0], 2 bits to select the dither dwell time.
#define DD_1ms              0x0000                  // default
#define DD_2ms              0x0040
#define DD_5ms              0x0080
#define DD_10ms             0x00C0
//==========================================================
// DS[3 0], 3-bit integer to select the dither steps.
//==============================================================================
#define Bridge_dead_time    0x1000
//==============================================================================
// SA[1 0], 2-bit integer to set sense amp gain.
#define SA_Gain_2_5         0x0000                  // default
#define SA_Gain_5           0x0080
#define SA_Gain_10          0x0100
#define SA_Gain_20          0x0180
//==========================================================
// DT[5 0], a 6-bit integer to set the dead time.



//==============================================================================
#define Gate_drive1         0x1800
//==============================================================================
// IR1[3 0], 4 bits to set turn-on current 1.
// IR2[3 0], 4 bits to set turn-on current 2.
//==============================================================================
#define Gate_drive2         0x2000
//==============================================================================
// IF1[3 0], 4 bits to set turn-off current 1.
// IRF2[3 0], 4 bits to set turn-off current 2.
//==============================================================================
#define Gate_drive3         0x2800
//==============================================================================
// TRS[3 0], 4-bit integer to set turn-on time.
// TFS[3 0], 4-bit integer to set turn-off time.
//==============================================================================
#define Current_limit       0x3000
//==============================================================================
// OBT[4 0], 5-bit integer to set the current limit blank time.
// VIL[3 0], 4-bit integer to set the current limit scale.
//==============================================================================
#define VDS_Current_Sense   0x3800
//==============================================================================
// MIT[1 0], 2-bit integer to set sense amp maximum threshold.
#define MIT_200mv           0x0000                  // default
#define MIT_100mv           0x0100
#define MIT_50mv            0x0200
#define MIT_25mv            0x0300
//==========================================================
// VT[5 0], 6-bit integer to set the VDS limit.
//==============================================================================
#define VDS                 0x4000
//==============================================================================
// VDQ, selects VDS qualifier mode.
#define VDQ_Debounce        0x0000                  // default
#define VDQ_Blank           0x0100
//==========================================================
// VQT[5 0], 6-bit integer to set the VDS qualifier time.
//==============================================================================
#define Watchdog1           0x4800
//==============================================================================
// WM[4 0], 5-bit integer to set the minimum watchdog time.
//==============================================================================
#define Watchdog2           0x5000
//==============================================================================
// WC[3 0], 4-bit integer to set watchdog cycle count.
//==========================================================
// WW[4 0], 5-bit integer to set the watchdog window time.
//==============================================================================
#define Commutation1        0x5800
//==============================================================================
// CP[3 0], 4-bits to set the steady-state phase control proportional gain.
//==========================================================
// CI[3 0], 4-bits to set the steady-state phase controller integral gain.
//==============================================================================
#define Commutation2        0x6000
//==============================================================================
// CPT[3 0], 4-bits to set the transient phase control proportional gain.
//==========================================================
// CIT[3 0], 4-bits to set the transient phase controller integral gain.
//==============================================================================
#define BEMF1               0x6800
//==============================================================================
// BW[4 0], 5-bit integer to set the BEMF detect window.
//==============================================================================
#define BEMF2               0x7000
//==============================================================================
// BS[1 0], 2 bits to select the number of BEMF samples.
#define BS_1                0x0000                  // default
#define BS_2                0x0020
#define BS_3                0x0040
#define BS_6                0x0050
//==========================================================
// BF[3 0], 4 bits to select the windmill BEMF filter time.
#define BF_disable          0x0000
#define BF_200us            0x0002                  // default
#define BF_400us            0x0004
#define BF_600us            0x0006
#define BF_800us            0x0008
#define BF_1ms              0x000A
#define BF_2ms              0x000C
#define BF_4ms              0x000E
#define BF_5ms              0x0010
#define BF_6ms              0x0012
#define BF_10ms             0x0014
#define BF_12ms             0x0016
#define BF_14ms             0x0018
#define BF_16ms             0x001A
#define BF_18ms             0x001C
#define BF_20ms             0x001E
//==============================================================================
#define Startup1            0x7800
//==============================================================================
// HT[3 0], a 4-bit integer to set the time of the initial alignment.
//==========================================================
// HD[3 0], a 4-bit integer to set the PWM duty cycle applied during the alignment time.
//==============================================================================
#define Startup2            0x8000
//==============================================================================
// STM, enables the coast function during startup.
#define STM_Coast_disabled  0x0000                  // default
#define STM_Coast_enabled   0x0200
//==========================================================
// RSC, enables restart after loss of synchronization.
#define RSC_No_restart      0x0000                  // default
#define RSC_restart         0x0100
//==========================================================
// KM[3 0], 4-bit integer to set the motor constant.
//==========================================================
// HR[1 0], 2 bits to select the alignment duty cycle ramp time.
#define HR_0                0x0000                  // default
#define HR_25               0x0002
#define HR_50               0x0004
#define HR_100              0x0006
//==============================================================================
#define Startup3            0x8800
//==============================================================================
// WIN, enables windmill detection at start-up
#define WIN_disabled        0x0000
#define WIN_enabled         0x0100
//==========================================================
// WMF[2 0], 3-bit integer to select the minimum windmill detection frequency.
//==========================================================
// WBD[3 0], 4-bit integer to select windmill brake duty cycle.
//==============================================================================
#define Startup4            0x9000
//==============================================================================
// SF2[3 0], a 4-bit integer to set final start frequency.
// SF1[3 0], a 4-bit integer to set initial start frequency.
//==============================================================================
#define Startup5            0x9800
//==============================================================================
// SD2[3 0], a 4-bit integer to set initial start PWM duty.
// SD1[3 0], a 4-bit integer to set final start PWM duty.
//==============================================================================
#define Startup6            0xA000
//==============================================================================
// STS[3 0], a 4-bit integer to set the start time step.
// SFS[3 0], a 4-bit integer to set the start frequency step.
//==============================================================================
#define Speed_control_loop1 0xA800
//==============================================================================
// SGL[5 0], a 5-bit integer to set the speed control loop acceleration limit.
// SG[3 0], a 4-bit integer to set the speed control loop proportional gain.
//==============================================================================
#define Speed_control_loop2 0xB000
//==============================================================================
// DV[1 0], 2 bits to select the voltage compensation level.
#define DV_Disable          0x0000
#define DV_0_12V            0x0100                  // default
#define DV_24V              0x0200
#define DV_1_12V            0x0300
//==========================================================
// DF[1 0], 2 bits to select the deceleration factor.
#define DF_1                0x0000                  // default
#define DF_2                0x0040
#define DF_5                0x0080
#define DF_10               0x00C0
//==========================================================
// SR[2 0], 3 bits to select the speed control resolution.
//==============================================================================
#define Speed_control_loop3 0xB800
//==============================================================================
// SL[3 0], 4 bits to select the underspeed threshold.
// SH[3 0], 4 bits to select the overspeed threshold.
//==============================================================================
#define NVM                 0xC000
//==============================================================================
// SAV[1 0], controls and reports saving the register contents to the NVM.
//==============================================================================
#define System_Functions    0xC800
//==============================================================================
// ESF, the enable stop on fault bit that defines the action taken when a fault is detected.
#define ESF_No_stop_on_fail 0x0000
#define ESF_stop_on_fail    0x0200                  // default
//==========================================================
// VLR, selects logic regulator voltage.
#define VLR_3_3V            0x0000                  // default
#define VLR_5V              0x0100
//==========================================================
// VRG, selects gate drive regulator voltage.
#define VRG_8V              0x0000
#define VRG_11V             0x0080                  // default
//==========================================================
// OPM, selects the stand-alone operating mode.
#define OPM_SPI_only        0x0000                  // default
#define OPM_Stand_alone     0x0040
//==========================================================
// LWK, selects the wake-up mode.
#define LWK_PWM_Wake        0x0000                  // default
#define LWK_LIN_Wake        0x0020
//==========================================================
// IPI, selects the sense of the PWM input.
#define IPI_Active_high     0x0000                  // default
#define IPI_Active_low      0x0010
//==========================================================
// DIL, disables current limit for speed modes.
#define DIL_Enabled         0x0000                  // default
#define DIL_Disabled        0x0008
//==========================================================
// CM[1 0], 2 bits to select the required control mode.
#define CM_Closed_loop_s1   0x0000                  // default
#define CM_Closed_loop_s2   0x0002
#define CM_Closed_loop_c    0x0004
#define CM_Open_loop        0x0006
//==============================================================================
#define Phase_advance 0xD000
//==============================================================================
// PAM, selects the phase advance mode.
#define PAM_Manual          0x0000                  // default
#define PAM_Automatic       0x0200
//==========================================================
// KIP[1 0], sets the auto phase advance control gain.
//==========================================================
// PA[5 0], a 6-bit integer to set the phase advance.
//==============================================================================
#define Motor_function      0xD800
//==============================================================================
// LEN, controls LIN standby/active state.
#define LEN_Standby         0x0000                  // default
#define LEN_Active          0x0100
//==========================================================
// GTS, initiates go-to-sleep function.
//==========================================================
// OVM[1 0], selects overmodulation level.
#define OVM_100             0x0000                  // default
#define OVM_1125            0x0020
#define OVM_125             0x0040
#define OVM_150             0x0060
//==========================================================
// DRM, selects drive mode.
#define DRM_Sinusoidal      0x0000                  // default
#define DRM_Trapezoidal     0x0010
//==========================================================
// BRK, brake control.
#define BRK_disabled        0x0000                  // default
#define BRK_enabled         0x0008
//==========================================================
// DIR, direction control.
#define DIR_Forward         0x0000                  // default
#define DIR_Reverse         0x0004
//==========================================================
// RUN, enables the motor to start and run.
#define RUN_Disable         0x0000                  // default
#define RUN_Start           0x0002
//==============================================================================
#define Fault_mask          0xE000
//==============================================================================
#define WD_permitted        0x0000
#define WD_disabled         0x0200                  // default
//==========================================================
#define LOS_permitted       0x0000                  // default
#define LOS_disabled        0x0100
//==========================================================
#define OT_permitted        0x0000                  // default
#define OT_disabled         0x0080
//==========================================================
#define TW_permitted        0x0000                  // default
#define TW_disabled         0x0040
//==========================================================
#define VSU_permitted       0x0000                  // default
#define VSU_disabled        0x0020
//==========================================================
#define VRU_permitted       0x0000                  // default
#define VRU_disabled        0x0010
//==========================================================
#define VLU_permitted       0x0000                  // default
#define VLU_disabled        0x0008
//==========================================================
#define BU_permitted        0x0000                  // default
#define BU_disabled         0x0004
//==========================================================
#define VO_permitted        0x0000                  // default
#define VO_disabled         0x0002
//==============================================================================
#define Readback_select     0xE800
//==============================================================================
// DGS[1 0], 2 bits to select output on the DIAG terminal.
#define DGS_1               0x0000                  // default;Active low fault flag
#define DGS_2               0x0100                  // FG; high when motor is stationary
#define DGS_3               0x0200                  // Pulse output; high when no fault present
#define DGS_4               0x0300                  // Pulse output; FG when no fault present
//==========================================================
// DSR, disables reset on serial transfer.
#define DSR_Enabled         0x0000                  // default
#define DSR_Disabled        0x0080
//==========================================================
// LBR, selects select LIN baud rate.
#define LBR_10KHz           0x0000                  // default
#define LBR_20KHz           0x0020
//==========================================================
// CKS, selects divided system clock on SDO terminal.
#define CKS_1               0x0000
#define CKS_2               0x0010
//==========================================================
// RBS[2 0], 3 bits to select the data that will be read from egister 31.
#define RBS_0               0x0000                  // default; Diagnostic register
#define RBS_1               0x0002                  // Motor speed
#define RBS_2               0x0004                  // Average supply current
#define RBS_3               0x0006                  // Supply voltage
#define RBS_4               0x0008                  // Chip temperature
#define RBS_5               0x000A                  // Demand input
#define RBS_6               0x000C                  // Applied bridge peak duty cycle
#define RBS_7               0x000E                  // Applied phase advance
//==============================================================================
#define Demand_input        0xF000//(Write only)
//==============================================================================
// DI[9 0], 10-bit integer providing the demand input for the selected control mode.
//==============================================================================
#define Data_acquisition    0xF800//(Read only)
//==============================================================================
// DO[9 0], 10 bits providing the internal monitor data selected by RBS[2 0].

*/





/*
//  不適用 xc8 , bit 無法跨 byte , 例 DW(4bits)..2bit在low byte , 2bit 在hight byte
//==============================================================================
// register struct define
//------------------------------------------------------------------------------
// Register 0: PWM Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned PW     :6;
		unsigned PMD    :1;
		unsigned reserve:1;
		unsigned MOD    :1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg00;
//------------------------------------------------------------------------------
// Register 1: PWM Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned DP     :3;
		unsigned DD     :2;
		unsigned DW     :4;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg01;
//------------------------------------------------------------------------------
// Register 2: Bridge and Sense Amp Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned DT     :6;
		unsigned SA     :2;
		unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg02;
//------------------------------------------------------------------------------
// Register 3: Gate Drive Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned IR2    :4;
		unsigned IR1    :4;
		unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg03;
//------------------------------------------------------------------------------
// Register 4: Gate Drive Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned IF2    :4;
		unsigned IF1    :4;
		unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg04;
//------------------------------------------------------------------------------
// Register 5: Gate Drive Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned TFS    :4;
		unsigned TRS    :4;
		unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg05;
//------------------------------------------------------------------------------
// Register 6: Current Limit Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned VIL    :4;
		unsigned OBT    :5;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg06;
//------------------------------------------------------------------------------
// Register 7: VDS Monitor and Sense Amp Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned VT     :6;
        unsigned reserve:1;
		unsigned MIT    :2;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg07;
//------------------------------------------------------------------------------
// Register 8: VDS Monitor Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned VQT    :6;
        unsigned reserve:1;
		unsigned VDQ    :1;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg08;
//------------------------------------------------------------------------------
// Register 9: Watchdog Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned WM     :5;
        unsigned reserve:4;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg09;
//------------------------------------------------------------------------------
// Register 10: Watchdog Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned WW     :5;
        unsigned WC     :4;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg10;
//------------------------------------------------------------------------------
// Register 11: Commutation Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned CI     :4;
        unsigned CP     :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg11;
//------------------------------------------------------------------------------
// Register 12: Commutation Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned CIT    :4;
        unsigned CPT    :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg12;
//------------------------------------------------------------------------------
// Register 13: BEMF Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned BW     :5;
        unsigned reserve:4;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg13;
//------------------------------------------------------------------------------
// Register 14: BEMF Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned BF     :4;
        unsigned BS     :2;
        unsigned reserve:3;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg14;
//------------------------------------------------------------------------------
// Register 15: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned HD     :5;
        unsigned HT     :4;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg15;
//------------------------------------------------------------------------------
// Register 16: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned HR     :2;
        unsigned KM     :4;
        unsigned reserve:1;
        unsigned RSC    :1;
        unsigned STM    :1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg16;
//------------------------------------------------------------------------------
// Register 17: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned WBD    :4;
        unsigned WMF    :3;
        unsigned WIN    :1;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg17;
//------------------------------------------------------------------------------
// Register 18: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SF1    :4;
        unsigned SF2    :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg18;
//------------------------------------------------------------------------------
// Register 19: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SD1    :4;
        unsigned SD2    :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg19;
//------------------------------------------------------------------------------
// Register 20: Startup Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SFS    :4;
        unsigned STS    :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg20;
//------------------------------------------------------------------------------
// Register 21: Speed Control Loop Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SG     :4;
        unsigned SGL    :5;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg21;
//------------------------------------------------------------------------------
// Register 22: Speed Control Loop Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SR     :3;
        unsigned reserve:2;
        unsigned DF     :2;
        unsigned DV     :2;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg22;
//------------------------------------------------------------------------------
// Register 23: Speed Control Loop Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned SH     :4;
        unsigned SL     :4;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg23;
//------------------------------------------------------------------------------
// Register 24: Write NVM Control
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned reserve:7;
        unsigned SAV    :2;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg24;
//------------------------------------------------------------------------------
// Register 25: System Configuration
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned CM     :2;
        unsigned DIL    :1;
        unsigned IPI    :1;
        unsigned LWK    :1;
        unsigned OPM    :1;
        unsigned VRG    :1;
        unsigned VLR    :1;
        unsigned ESF    :1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg25;
//------------------------------------------------------------------------------
// Register 26: Phase Advance Select
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned PA     :6;
        unsigned KIP    :2;
        unsigned PAM    :1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg26;
//------------------------------------------------------------------------------
// Register 27: Motor Function Control
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned RUN    :1;
        unsigned DIR    :1;
        unsigned BRK    :1;
        unsigned DRM    :1;
        unsigned OVM    :2;
        unsigned GTS    :1;
        unsigned LEN    :1;
        unsigned reserve:1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg27;
//------------------------------------------------------------------------------
// Register 28: Mask Register
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned VO     :1;
        unsigned BU     :1;
        unsigned VLU    :1;
        unsigned VRU    :1;
        unsigned VSU    :1;
        unsigned TW     :1;
        unsigned OT     :1;
        unsigned LOS    :1;
        unsigned WD     :1;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg28;
//------------------------------------------------------------------------------
// Register 29: Readback Select
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned RBS    :3;
        unsigned CKS    :1;
        unsigned LBR    :1;
        unsigned reserve:1;
        unsigned DSR    :1;
        unsigned DGS    :2;
		unsigned WR     :1;
		unsigned ADDR   :5;
	}bits;
}reg29;
//------------------------------------------------------------------------------
// Register 30: Demand input (Write Only)
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned DI     :10;
		unsigned ADDR   :5;
	}bits;
}reg30;
//------------------------------------------------------------------------------
// Register 31: Readback Register (Read Only)
union {
	unsigned int word;		
	unsigned char byte[2];
	struct{
		unsigned p      :1;
		unsigned DO     :10;
		unsigned ADDR   :5;
	}bits;
}reg31;
*/



#endif	/* A964_DEF_H */

