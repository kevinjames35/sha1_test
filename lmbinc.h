/*
 * lmbinc.h
 *
 *  Created on: Nov 27, 2017
 *      Author: legend
 */

#ifndef LMBINC_H_
#define LMBINC_H_
#include <stdint.h>

/*** Data Type Define ***/
#if 0
#define int8_t  		char
#define int16_t 		short
#define int32_t 		int
#define int64_t  		long long
#define uint8_t  		unsigned char
#define uint16_t 		unsigned short
#define uint32_t 		unsigned int
#define uint64_t 		unsigned long long
#endif

/*** Return Value ***/
#define ERR_Success		0
#define ERR_Error		0xFFFFFFFF 	/*(-1)*/
#define ERR_NotExist		0xFFFFFFFE	/*(-2)*/
#define ERR_NotOpened		0xFFFFFFFD	/*(-3)*/
#define ERR_Invalid		0xFFFFFFFC	/*(-4)*/
#define ERR_NotSupport		0xFFFFFFFB	/*(-5)*/
#define ERR_BusyInUses		0xFFFFFFFA	/*(-6)*/
#define ERR_BoardNotMatch	0xFFFFFFF9	/*(-7)*/
#define ERR_DriverNotLoad	0xFFFFFFF8	/*(-8)*/
/*** IPMI access error ***/
#define ERR_IPMI_IDLESTATE	0xFFFFFEFF	/*(-257)*/
#define ERR_IPMI_WRITESTATE	0xFFFFFEFE	/*(-258)*/
#define ERR_IPMI_READSTATE	0xFFFFFEFD	/*(-259)*/
#define ERR_IPMI_IBF0		0xFFFFFEFC	/*(-260)*/
#define ERR_IPMI_OBF1		0xFFFFFEFB	/*(-261)*/

/*** Timer Function ***/
#define BASE_SECOND		1
#define BASE_MINUTE		2

/*** DIO Function ***/
#define DIRECTION_INPUT		0
#define DIRECTION_OUTPUT	1

#define DISABLE		0
#define	ENABLE		1

/* LAN Bypass Controller Slot define */
#define	SLOT_ONBOARD		0
#define	SLOT_INDEX_1		1
#define	SLOT_INDEX_2		2
#define	SLOT_INDEX_3		3
#define	SLOT_INDEX_4		4
#define	SLOT_INDEX_5		5
#define	SLOT_INDEX_6		6
#define	SLOT_INDEX_7		7
#define	SLOT_INDEX_8		8

/* LAN Slot modules type */
#define LAN_TYPE_UNKNOWN	0
#define LAN_TYPE_COPPER		1
#define LAN_TYPE_FIBER		2


/* LAN Bypass modules bit define */
#define LBP_MODBIT_SystemOff	0x01
#define LBP_MODBIT_JustOn	0x02
#define	LBP_MODBIT_Runtime	0x04
#define	LBP_MODBIT_TIMER1	0x08
#define	LBP_MODBIT_TIMER2	0x10
#define	LBP_MODBIT_TIMER3	0x20
#define	LBP_MODBIT_Disconnect	0x40


/* LAN Bypass WDT Configyre: action item */
#define PAIR_NOASSIGN		0
#define	PAIR_BYPASS_ENABLE	1
#define	PORT_DISCONNECT		2

/* HWM Monitor Intrusion Item Define (bit) */
#define INTRMSG_CASEOPEN	0x00000001

/* HWM Fan sequence */
#define FAN_A		1
#define FAN_B		2


/* Software Reset Button Intrusion Item Define (bit) */
#define INTRMSG_SWRESET		0x00000001
/* Power Supply Intrusion Item Define (bit) */
#define INTRMSG_PSU1_INPUT	0x00000001
#define INTRMSG_PSU2_INPUT	0x00000002
#define INTRMSG_PSU1_TEMP	0x00000004
#define INTRMSG_PSU2_TEMP	0x00000008
#define INTRMSG_PSU1_VOUT	0x00000010
#define INTRMSG_PSU2_VOUT	0x00000020
#define INTRMSG_PSU1_IOUT	0x00000040
#define INTRMSG_PSU2_IOUT	0x00000080
#define INTRMSG_PSU1_FAN	0x00000100
#define INTRMSG_PSU2_FAN	0x00000200

/* WDT Type define */
#define WDT_TYPE_UNKNOWN	0
#define WDT_TYPE_SIO		1
#define WDT_TYPE_TCO		2

/* PSU Watts Select */
#define PSU_WATTS_INPUT		0
#define PSU_WATTS_OUTPUT	1

/* PSU Status Bit define */

#define PSU_FAULT_NONEOFABOVE	0x0001
#define PSU_FAULT_COMM		0x0002
#define PSU_FAULT_TEMP		0x0004
#define PSU_FAULT_VIN_UV	0x0008
#define PSU_FAULT_IOUT_OC	0x0010
#define PSU_FAULT_VOUT_OV	0x0020
#define PSU_FAULT_UINTOFF	0x0040
#define PSU_FAULT_UINTBUSY	0x0080
#define PSU_FAULT_UNKNOWN	0x0100
#define PSU_FAULT_OTHERS	0x0200
#define PSU_FAULT_FAN		0x0400
#define PSU_FAULT_POWERGOOD	0x0800
#define PSU_FAULT_MFRSPEC	0x1000
#define PSU_FAULT_INPUT		0x2000
#define PSU_FAULT_IOUT		0x4000
#define PSU_FAULT_VOUT		0x8000

typedef struct DEF_DLL_VERSION {
	uint16_t uwDllMajor;
	uint16_t uwDllMinor;
	uint16_t uwDllBuild;
	int8_t   strPlatformID[15];
	uint16_t uwBoardMajor;
	uint16_t uwBoardMinor;
	uint16_t uwBoardBuild;
}DLL_VERSION;

typedef struct DEF_WDT_INFO {
	uint8_t  ubType;
	uint16_t uwCountMax;
	uint8_t	 ubMinuteSupport;
}WDT_INFO;

typedef struct DEF_LBPDEV_INFO {
	uint8_t  ubSlotIndex;		
	uint8_t	 ubType;		//copper or fiber
	uint8_t  ubVersion[2];		//major & minor
	uint8_t	 ubModules;		//modules supprts	
	uint8_t  ubSystemOff_Pairs;	//maximum pairs
	uint8_t  ubJustOn_Pairs;
	uint8_t  ubRuntime_Pairs;
	uint16_t uwTimer1_MaxSec;	//WDT maximum value
	uint16_t uwTimer2_MaxSec;
	uint16_t uwTimer3_MaxSec;
}LBPDEV_INFO;

typedef struct DEF_PAIRS_STATUS {
	uint8_t ubSlotIndex;
	uint8_t ubSystemOff_PairsBypass;
	uint8_t ubJustOn_PairsBypass;
	uint8_t ubRuntime_PairsBypass;
	uint8_t ubJustOn_PortsDisconnect;
	uint8_t ubRuntime_PortsDisconnect;	
}PAIRS_STATUS;

typedef struct DEF_LBP_TIMERCFG {
	uint8_t	 ubTimerNo;	//timer number
	uint16_t uwTimeSec;	//timer value by second
	uint8_t  ubAction;	//timeout action method
	uint8_t  ubEffect;	//assign pairs or ports for action
}LBP_TIMERCFG;

typedef struct DEF_INTRUSION_TIME {
	uint16_t uwYear;
	uint8_t	 ubMonth;
	uint8_t	 ubDay;
	uint8_t	 ubHour;
	uint8_t	 ubMinute;
	uint8_t	 ubSecond;
}INTRUSION_TIME;

/*** Intrusion Callback Fucntion ***/
typedef struct DEF_INTRUSION_MSG {
	uint32_t udwOccurItem;		//Occure item
	uint32_t udwStatus;		//Occure status
	INTRUSION_TIME stuTime;		//Occure time
}INTRUSION_MSG;
typedef void (*INTRUSION_CALLBACK)(INTRUSION_MSG);

/*** GPI Callback Fucntion ***/
typedef struct DEF_GPI_MSG {
	uint8_t  ubGroup;		//GPI group
	uint32_t udwGpis;		//Occure Gpis
	uint32_t udwStatus;		//Occure status
	INTRUSION_TIME stuTime;		//Occure time
}GPI_MSG;
typedef void (*GPI_CALLBACK)(GPI_MSG);

typedef struct DEF_LCM_INFO {
	uint16_t uwModeNo;
	uint16_t uwVersion;
	uint32_t udwBaudrate;
}LCM_INFO;

/*** LCM Keys Callback Fucntion ***/
typedef struct DEF_LCMKEY_MSG {
	uint8_t ubKeys;			//Occure Keys
	uint8_t ubStatus;		//Occure status
	INTRUSION_TIME stuTime;		//Occure time
}LCMKEY_MSG;
typedef void (*LCMKEY_CALLBACK)(LCMKEY_MSG);

/*** Power Supply ***/
typedef struct DEF_PSU_INFO {
	uint8_t	 ubPsuNo;
	uint8_t	 strubMfrId[33];
	uint8_t	 strubMfrModel[33];
	uint8_t	 strubMfrSerial[33];
	uint8_t	 strubMfrRevision[33];
}PSU_INFO;

typedef struct DEF_PSU_WATTS {
	uint8_t	ubPsuNo;
	uint8_t	ubInOut;
	float	fVolts;
	float	fAmperes;	
	float	fWatts;
}PSU_WATTS;

typedef struct DEF_PSU_SENSORS {
	uint8_t	 ubPsuNo;
	float	 fTemp_1;
	float	 fTemp_2;	
	uint16_t uwFanRpm;
}PSU_SENSORS;

/******* Power Over Ethernet  define ******/
typedef	struct DEF_POECLASS_STATUS {
	int8_t	bEnLTPoE;
	int8_t	bClassNo;
	int8_t	bDetect;
	int8_t	strClassMsg[15];
	int8_t	strDetectMsg[15];
}POECLASS_STATUS;


/***********  Ignition Device  **************/
/*** Ignition functions ***/
#define	FUNC_STARTUPWDT	0x0001
#define FUNC_LOWPOWER	0x0002
#define FUNC_IGNGPI	0x0004
#define FUNC_IGNGPO	0x0008
#define FUNC_IGNPOE	0x0010
#define FUNC_HEATER	0x0020	

/** Power Status **/
#define STAT_CLOSEUP		0
#define STAT_POWERON_DELAY	1
#define STAT_WAIT_STARTUP	2
#define STAT_STARTUP		3
#define STAT_SHUTDOWN_DELAY	4
#define STAT_SHUTTING_DOWN	5
#define STAT_LOWPOWER_DELAY	6
#define STAT_UNKNOWN		255

/** LMB_IGN_GetParamRange define **/
#define PARAM_KeyOnDelayTime		1
#define PARAM_StartUpTime		2
#define PARAM_StartupWdtTime		3
#define PARAM_MaxFailStartupCnt 	4
#define PARAM_LowmVolt			5
#define PARAM_HighmVolt			6
#define PARAM_ConfirmTime		7
#define PARAM_MaxFailShutdownCnt	8
#define PARAM_KeyOffDelayTime		9
#define PARAM_ShutdownTime		10
#define PARAM_mCelsiusMin		11
#define PARAM_mCelsiusMax		12
#define PARAM_Unknown			(PARAM_mCelsiusMax+1)

/*** Wake-up pin define ***/
#define WAKE_IGN_DI0	0x01
#define WAKE_IGN_DI1	0x02
#define WAKE_3G_MODULE	0x04



typedef struct DEF_STARTUP_SCHEME {
	int32_t dwMaxFailStartupCnt;
	int32_t dwStartupWdtTime;
	int32_t dwKeyOnDelayTime;
	int32_t dwStartUpTime;
}STARTUP_SCHEME;

typedef struct DEF_LOWPOWER_SCHEME {
	int32_t dwDisabled;
	int32_t dwLowmVolt;		//mV
	int32_t dwHighmVolt;		//mV
	int32_t dwConfirmTime;		//second
}LOWPOWER_SCHEME;

typedef struct DEF_SHUTDOWN_SCHEME {
	int32_t dwMaxFailShutdownCnt;
	int32_t dwKeyOffDelayTime;
	int32_t dwShutdownTime;
}SHUTDOWN_SCHEME;

typedef struct DEF_HEATER_SCHEME {
	int32_t dwmCelsiusMin;		//mCelsius
	int32_t dwmCelsiusMax;		//mCelsius
}HEATER_SCHEME;

typedef struct DEF_IGN_VERSION {
	int8_t strbDeviceID[30];
	int8_t strbVersion[30];
}IGN_VERSION;

typedef struct DEF_IGN_RUNTIME_INFO {
	int32_t dwPowerState;
	int32_t dwInputVoltage;		//mV
	int32_t dwFailShutdownCnt;
	int32_t dwFailStartupCnt;
	int32_t dwShutdownFalg;
}IGN_RUNTIME_INFO;

typedef struct DEF_VALUE_RANGE {
	int32_t	dwMinimum;
	int32_t dwMaximum;
	int32_t dwDefault;
}VALUE_RANGE;


/***********  GPS Device  **************/
typedef struct  DEF_RMC_DATA {
        int8_t 	strLongitude[20];		//‘E’ = East; ‘W’ = West, Longitude in dddmm.mmmm format.
        int8_t 	strLatitude[20]; 		//‘N’ = North; ‘S’ = South, Latitude in dddmm.mmmm format.
        int8_t 	strSpeedKnots[10];    		//Speed over ground in knots (000.0 ~ 999.9)
        int8_t	strStatus[10];			//Status, ‘V’ = Navigation receiver warning, ‘A’ = Data Valid
        int8_t	strUTCTime[15];			//UTC time in hhmmss.sss format (000000.000 ~ 235959.999)
	int8_t	strUTCDate[15];			//UTC date of position fix, ddmmyy format
        int8_t	strCourse[10];			//Course over ground in degrees (000.0 ~ 359.9)
}RMC_DATA;

typedef struct  DEF_GGA_DATA {
        int8_t 	strLongitude[20];		//‘E’ = East, ‘W’ = West, Longitude in dddmm.mmmm format.
        int8_t 	strLatitude[20]; 		//‘N’ = North, ‘S’ = South,  Latitude in ddmm.mmmm format.
	int8_t	strAltitude[10];		//Mean sea level altitude (-9999.9 ~ 17999.9) in meter
	int8_t 	strGeoidalSeparation[10];	//In meter
	int8_t	strHDOP[5];			//Horizontal dilution of precision, (00.0 ~ 99.9)
	int8_t	strQuality[5];			//GPS quality indicator
						//	0: position fix unavailable
						//	1: valid position fix, SPS mode
						//	2: valid position fix, differential GPS mode
	int8_t	strUTCTime[15];			//UTC of position in hhmmss.sss format, (000000.000 ~ 235959.999)
        int8_t	strSatelitesUsed[5];		//Number of satellites in use, (00 ~ 24)
}GGA_DATA;

typedef struct  DEF_VTG_DATA {
        int8_t 	strTrueCourse[10];		//Course over ground, degrees True (000.0 ~ 359.9) 
        int8_t 	strMagneticCourse[10]; 		//Course over ground, degrees Magnetic (000.0 ~ 359.9)
	int8_t	strSpeedKnots[10];		//Speed over ground in knots (000.0 ~ 999.9)
	int8_t	strSpeedKm[10];			//Speed over ground in kilometers per hour (0000.0 ~ 1800.0)
	int8_t	strMode[5];			//Mode indicator, ‘N’ = not valid, 
						//		  ‘A’ = Autonomous mode
						//		  ‘D’ = Differential mode
						//		  ‘E’ = Estimated (dead reckoning) mode
}VTG_DATA;

typedef struct  DEF_GLL_DATA {
 	int8_t 	strLongitude[20];		//‘E’ = East, ‘W’ = West, Longitude in dddmm.mmmm format.
        int8_t 	strLatitude[20]; 		//‘N’ = North, ‘S’ = South,  Latitude in ddmm.mmmm format.
	int8_t	strUTCTime[15];			//UTC of position in hhmmss.sss format, (000000.000 ~ 235959.999)
	int8_t	strStatus[10];			//Status, ‘V’ = Navigation receiver warning, ‘A’ = Data Valid
}GLL_DATA;

typedef struct  DEF_GSA_DATA {
 	int8_t 	strMode[5];			//‘M’ = Manual forced 2D or 3D, ‘A’ = Automatic switch 2D/3D
        int8_t 	strType[5]; 			//fix tyupe, ‘1’ = fix not available, '2' = 2D, '3' = 3D 
	uint8_t	aryubSatelliteID[24];		//Satellite used ID list, Maximally 24 satellites
	int8_t	strPDOP[10];			//Position dilution of precision (00.0 to 99.9)
	int8_t	strHDOP[10];			//Horizontal dilution of precision (00.0 to 99.9)
	int8_t	strVDOP[10];			//Vertical dilution of precision (00.0 to 99.9)
}GSA_DATA;

typedef struct DEF_SATELITE_INFO {
	int8_t	strSateliteID[5];		
	int8_t	strElevation[5];			//Satellite elevation in degrees, (00 ~ 90)
	int8_t	strAzimuth[5];				//Satellite azimuth angle in degrees, (000 ~ 359 )
	int8_t	strSNR[5];				//Signal to Noise Ratio, in dB (00 ~ 99), Null when not tracking
}SATELITE_INFO;

typedef struct  DEF_GSV_DATA {
	int8_t		bGPStotal;			//total numbers of GPS satellites
	int8_t		bGLONASStotal;			//total numbers of GLONASS satellites
	SATELITE_INFO	stuGPS[12];			//GPS satellites
	SATELITE_INFO	stuGLONASS[12];			//GLONASS satellites
}GSV_DATA;


/***********  G-Sensor Device ADXL345  **************/
#define	GSR_DEVID		0x00	//Device ID, read only
#define GSR_THRESH_TAP		0x1D	//Tap threshold
#define GSR_OFSX		0x1E	//X-axis offset
#define GSR_OFSY		0x1F	//Y-axis offset
#define GSR_OFSZ		0x20	//Z-axis offset
#define GSR_DUR			0x21	//Tap duration
#define GSR_LATENT		0x22	//Tap latency
#define GSR_WINDOW		0x23	//Tap window	
#define GSR_THRESH_ACT		0x24	//Activity threshold
#define GSR_THRESH_INACT	0x25	//Inactivity threshold
#define GSR_TIME_INACT		0x26	//Inactivity time
#define GSR_ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define GSR_THRESH_FF		0x28	//Free-fall threshold
#define GSR_TIME_FF		0x29	//Free-fall time
#define GSR_TAP_AXES		0x2A	//Axis control for single tap/double tap
#define GSR_ACT_TAP_STATUS	0x2B	//Source of single tap/double tap, read only
#define GSR_BW_RATE		0x2C	//Data rate and power mode control	
#define GSR_POWER_CTL		0x2D	//Power-saving features control
#define GSR_INT_ENABLE		0x2E	//Interrupt enable control
#define GSR_INT_MAP		0x2F	//Interrupt mapping control
#define GSR_INT_SOURCE		0x30	//Source of interrupts, read only
#define GSR_DATA_FORMAT		0x31	//Data format control
#define GSR_DATAX0		0x32	//X-Axis Data 0, read only
#define GSR_DATAX1		0x33	//X-Axis Data 1, read only
#define GSR_DATAY0		0x34	//Y-Axis Data 0, read only
#define GSR_DATAY1		0x35	//Y-Axis Data 1, read only
#define GSR_DATAZ0		0x36	//Z-Axis Data 0, read only
#define GSR_DATAZ1		0x37	//Z-Axis Data 1, read only
#define GSR_FIFO_CTL		0x38	//FIFO control
#define GSR_FIFO_STATUS		0x39	//FIFO status, read only

/*** G-Sensor X,Y,Z Axis ***/
typedef struct DEF_AXIS_RAWDATA {
	int16_t	wXaxis;
	int16_t	wYaxis;
	int16_t	wZaxis;
	int16_t wgRange;
}AXIS_RAWDATA;


/***********  CAN bus Module MT3647  **************/
typedef struct DEF_DRIVING_INFO { 
	uint8_t	 ubSpeedKm;
	uint8_t  ubAcceleratorPedal;	//Accelerator Pedal Position (APP)
	uint8_t  ubFuelLevel;	
	float 	 fTotalDistance;	//High Resolution Total Vehicle Distance
	int32_t  dwSERV;		//The distance which can be traveled
	uint8_t	 ubTachographKm ;	//tachograph vehicle speed
	float	 fBatteryVolt;	
}DRIVING_INFO;

typedef struct DEF_ENGINE_INFO {
	uint16_t uwEngineRPM;
	uint8_t	 ubEngineLoading;
	int16_t	 wCoolantTemp;		//Engine Coolant Temperature(ECT)
	int16_t	 wEIMT;			//Engine Intake Manifold 1 Temperature
	uint32_t udwTotalFuels;		//Engine Total Fuel used
	uint32_t udwTotalHours;		//Engine total hours of Operation
	uint16_t uwETBP	; 		//Engine Turbocharger Boost Pressure
	float	 fEFR;			//Engine Fuel Rate
	float 	 fEIFE;			//Engine Instantaneous Fuel Economy
	float	 fEODP;			//Engine Oil Filter Differential Pressure
}ENGINE_INFO;


/***************************************************************/
/*****   	    API functions  		****************/
/***************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif
/*** DLL functions group *******/
int32_t LMB_DLL_Init(void);
int32_t LMB_DLL_DeInit(void);
int32_t LMB_DLL_Version(DLL_VERSION* pstuDllVersion);
/*** Hardware Monitor Group ***/
int32_t LMB_HWM_DevInfo(uint32_t* pudwDevInfo);
int32_t LMB_HWM_GetCpuTemp(uint8_t ubCpuNo, float* pfTemperature);
int32_t LMB_HWM_GetSysTemp(uint8_t ubSysNo, float* pfTemperature);
int32_t LMB_HWM_GetVcore(uint8_t ubCpuNo, float* pfVoltage);
int32_t LMB_HWM_Get12V(float* pfVoltage);
int32_t LMB_HWM_Get5V(float* pfVoltage);
int32_t LMB_HWM_Get3V3(float* pfVoltage);
int32_t LMB_HWM_Get5Vsb(float* pfVoltage);
int32_t LMB_HWM_Get3V3sb(float* pfVoltage);
int32_t LMB_HWM_GetVbat(float* pfVoltage);
int32_t LMB_HWM_GetCpuFan(uint8_t ubCpuNo, uint16_t* puwRpm);
int32_t LMB_HWM_GetSysFan(uint8_t ubSysNo, uint16_t* puwRpm);
int32_t LMB_HWM_GetFanSpeed(uint8_t ubFanNo, uint16_t* puwRpm);
int32_t LMB_HWM_GetFanSpeedEx(uint8_t ubFanNo, uint16_t* puwRpm, uint8_t ubFanSeq);
int32_t LMB_HWM_GetPowerSupply(uint8_t ubPowerNo, uint16_t* puwVoltage);
int32_t LMB_HWM_CaseOpenStatus(uint8_t* pubStatus);
int32_t	LMB_HWM_IntrCallback(INTRUSION_CALLBACK pCallback, uint16_t uwmSec);
int32_t LMB_HWM_ClearCaseOpenStatus(void);
int32_t LMB_HWM_GetVDDR(int8_t bChannel, float* pfVoltage);
int32_t LMB_HWM_GetSensorReport(int32_t dwSensorID, int8_t* pbString);
int32_t LMB_HWM_GetSensorName(int32_t dwSensorID, int8_t* pbString);
/*** Watch Dog Timer ***/
int32_t LMB_WDT_QueryInfo(WDT_INFO* pstuWdtInfo);
int32_t LMB_WDT_Config(uint16_t uwCount, uint8_t ubTimeBase);
int32_t LMB_WDT_Start(void);
int32_t LMB_WDT_Stop(void);
int32_t LMB_WDT_Tick(void);
/*** General Purpose Input Output ***/
int32_t LMB_GPIO_GetInfo(uint8_t ubGroup, uint8_t* pubGpis, uint8_t* pubGpos);
int32_t LMB_GPIO_GpoWrite(uint8_t ubGroup, uint32_t udwValue);
int32_t	LMB_GPIO_GpoPinWrite(uint8_t ubGroup, uint8_t ubPinNo, uint8_t ubValue);
int32_t LMB_GPIO_GpoRead(uint8_t ubGroup, uint32_t* pdwValue);
int32_t LMB_GPIO_GpoPinRead(uint8_t ubGroup, uint8_t ubPinNo, uint8_t* pubValue);
int32_t LMB_GPIO_GpiRead(uint8_t ubGroup, uint32_t* pdwValue);
int32_t LMB_GPIO_GpiPinRead(uint8_t ubGroup, uint8_t ubPinNo, uint8_t* pubValue);
int32_t	LMB_GPIO_GpiCallback(GPI_CALLBACK pCallback, uint16_t uwmSec);
/*** System LED ***/
int32_t LMB_SLED_SetSystemLED(uint8_t ubLedMode);
int32_t LMB_SLED_GetSystemLED(uint8_t* pubLedMode);
/*** Software Reset Button ***/
int32_t LMB_SWR_GetStatus(uint8_t* pubStatus);
int32_t	LMB_SWR_IntrCallback(INTRUSION_CALLBACK pCallback, uint16_t uwmSec);
/*** LAN Bypass Control ***/
int32_t LMB_LBP_QueryDevices(uint16_t* puwSlotsDev);
int32_t LMB_LBP_DeviceInfo(uint8_t ubSlot, LBPDEV_INFO* pstuDevInfo);
int32_t LMB_LBP_FactoryReset(uint8_t ubSlot);
int32_t LMB_LBP_SaveConfig(uint8_t ubSlot);
int32_t LMB_LBP_GetPairsStatus(uint8_t ubSlot, PAIRS_STATUS* pstuPairsStatus);
int32_t LMB_LBP_SetPairBypass(uint8_t ubSlot, uint8_t ubPairNo, uint8_t ubEnable);
int32_t LMB_LBP_SetAllPairs(uint8_t ubSlot, uint8_t ubStatus);
int32_t LMB_LBP_SetJustOnPairs(uint8_t ubSlot, uint8_t ubStatus);
int32_t LMB_LBP_SetSystemOffPairs(uint8_t ubSlot, uint8_t ubStatus);
int32_t LMB_LBP_SetPortDisconnect(uint8_t ubSlot, uint8_t ubPortNo, uint8_t ubEnable);
int32_t LMB_LBP_SetAllPortsDisconnect(uint8_t ubSlot, uint8_t ubStatus);
int32_t LMB_LBP_TimerConfig(uint8_t ubSlot, LBP_TIMERCFG stuLbpTimerCfg);
int32_t LMB_LBP_TimerStart(uint8_t ubSlot, uint8_t ubTimerNo);
int32_t LMB_LBP_TimerStop(uint8_t ubSlot, uint8_t ubTimerNo);
int32_t LMB_LBP_TimerTick(uint8_t ubSlot, uint8_t ubTimerNo);
int32_t LMB_LBP_TimerStatus(uint8_t ubSlot, uint8_t ubTimerNo, uint8_t *pubStatus);
/*** Serial EEPROM ***/
int32_t LMB_EEP_QueryDevices(uint16_t* puwSlotsDev);
int32_t LMB_EEP_WriteByte(uint8_t ubDeviceNo, uint32_t udwAddr, uint8_t ubData);
int32_t LMB_EEP_WriteWord(uint8_t ubDeviceNo, uint32_t udwAddr, uint16_t uwData);
int32_t LMB_EEP_WriteDWord(uint8_t ubDeviceNo, uint32_t udwAddr, uint32_t udwData);
int32_t LMB_EEP_WriteBlock(uint8_t ubDeviceNo, uint32_t udwAddr, uint16_t uwLength, uint8_t* pubBlock);
int32_t LMB_EEP_ReadByte(uint8_t ubDeviceNo, uint32_t udwAddr, uint8_t* pubData);
int32_t LMB_EEP_ReadWord(uint8_t ubDeviceNo, uint32_t udwAddr, uint16_t* puwData);
int32_t LMB_EEP_ReadDWord(uint8_t ubDeviceNo, uint32_t udwAddr, uint32_t* pudwData);
int32_t LMB_EEP_ReadBlock(uint8_t ubDeviceNo, uint32_t udwAddr, uint16_t uwLength, uint8_t* pubBlock);
int32_t LMB_EEP_Erase(uint8_t ubDeviceNo);
/*** LCD Modules ***/
int32_t LMB_LCM_DeviceOpen(void);
int32_t LMB_LCM_DeviceClose(void);
int32_t LMB_LCM_DeviceInfo(LCM_INFO* pstuLcmInfo);
int32_t LMB_LCM_LightCtrl(uint8_t ubOnOff);
int32_t LMB_LCM_SetCursor(uint8_t ubColumn, uint8_t ubRow);
int32_t LMB_LCM_WriteString(int8_t* pstrubString);
int32_t LMB_LCM_DisplayClear(void);
int32_t LMB_LCM_Brightness(uint8_t ubBrightness);
int32_t LMB_LCM_Reset(void);
int32_t LMB_LCM_SetSpeed(uint32_t udwBaudrate);
int32_t LMB_LCM_WrapCtrl(uint8_t ubOnOff);
int32_t LMB_LCM_CursorModeCtrl(uint8_t ubcursorMode);
int32_t LMB_LCM_KeysStatus(uint8_t* pubKeys);
int32_t	LMB_LCM_KeysCallback(LCMKEY_CALLBACK pCallback, uint16_t uwmSec);
int32_t LMB_LCM_StartupMsg(uint8_t* pubMsg, uint8_t length);
int32_t LMB_LCM_SearchPort(int8_t* pbLcmPort, int32_t* pdwBaudrate);
int32_t LMB_LCM_OpenPort(int8_t* pbLcmPort, int32_t dwBaudrate);
/*** Redundant Power Supply Modules ***/
int32_t LMB_PSU_QueryDevices(uint16_t* puwPsuDev);
int32_t LMB_PSU_DeviceInfo(PSU_INFO* pstuPsuInfo);
int32_t LMB_PSU_SensorInfo(PSU_SENSORS* pstuPsuSensors);
int32_t LMB_PSU_WattsInfo(PSU_WATTS* pstuWatts);
int32_t LMB_PSU_Status(uint8_t ubPsuNo, uint16_t* puwStatus);
int32_t	LMB_PSU_IntrCallback(INTRUSION_CALLBACK pCallback, uint16_t uwmSec);
/*** Power over Ethernet ***/
int32_t LMB_POE_QueryDevices(uint32_t* pudwPorts);
int32_t LMB_POE_SetPortPower(uint8_t ubPort, uint8_t ubEnable);
int32_t LMB_POE_GetPortStatus(uint8_t ubPort, uint32_t* pudwStatus);
int32_t LMB_POE_GetClassStatus(uint8_t ubPort, POECLASS_STATUS* pstuClassStatus);
int32_t LMB_POE_GetMeasurement(uint8_t ubPort, int32_t* pmVolt, int32_t* puAmpere);
/*** Ignition functions groups ****/
int32_t LMB_IGN_OpenPort(int8_t* pbPortPath);
int32_t LMB_IGN_ClosePort(void);
int32_t LMB_IGN_VersionInfo(IGN_VERSION* pstuIgnVersion);
int32_t LMB_IGN_SetStartupScheme(STARTUP_SCHEME stuStartupScheme);
int32_t LMB_IGN_GetStartupScheme(STARTUP_SCHEME* pstuStartupScheme);
int32_t LMB_IGN_SetShutdownScheme(SHUTDOWN_SCHEME stuShutdownScheme);
int32_t LMB_IGN_GetShutdownScheme(SHUTDOWN_SCHEME* pstuShutdownScheme);
int32_t LMB_IGN_SetLowPowerScheme(LOWPOWER_SCHEME stuLowPowerScheme);
int32_t LMB_IGN_GetLowPowerScheme(LOWPOWER_SCHEME* pstuLowPowerScheme);
int32_t LMB_IGN_LoadFactorySetting(void);
int32_t LMB_IGN_SaveToDefault(void);
int32_t LMB_IGN_SystemShutdown(void);
int32_t LMB_IGN_SetWakeupInput(uint8_t ubInput);
int32_t LMB_IGN_GetWakeupInput(uint8_t* pubInput);
int32_t LMB_IGN_GetKeyOnStat(uint8_t* pubStat);
int32_t LMB_IGN_FuncSupport(uint16_t* puwFunc);
int32_t LMB_IGN_StopStartupWdt(void);
int32_t LMB_IGN_ReloadStartupWdt(int32_t dwSeconds);
int32_t LMB_IGN_RuntimeInfo(IGN_RUNTIME_INFO* pstuRuntimeInfo);
int32_t LMB_IGN_GetDigitalPins(uint32_t* pudwOutPins, uint32_t* pudwInPins);
int32_t LMB_IGN_SetDigitalOut(uint32_t udwPinAssign, uint32_t udwDigitalOut);
int32_t LMB_IGN_GetDigitalOut(uint32_t udwPinAssign, uint32_t* pudwDigitalOut);
int32_t LMB_IGN_GetDigitalIn(uint32_t udwPinAssign, uint32_t* pdwDigitalIn);
int32_t LMB_IGN_SetPoePower(uint32_t udwPortAssign, uint32_t udwPowerEnable);
int32_t LMB_IGN_GetPoePower(uint32_t udwPortAssign, uint32_t* pudwPowerStatus);
int32_t LMB_IGN_QueryPoePorts(uint32_t* pudwPorts);
int32_t LMB_IGN_SetHeaterScheme(HEATER_SCHEME stuHeaterScheme);
int32_t LMB_IGN_GetHeaterScheme(HEATER_SCHEME* pstuHeaterScheme);
int32_t LMB_IGN_GetParamRange(uint8_t ubParam, VALUE_RANGE* pstuValueRange);
int32_t	LMB_IGN_IntrCallback(INTRUSION_CALLBACK pCallback, uint16_t uwmSec);
/*** GPS ***/
int32_t LMB_GPS_OpenPort(int8_t* pbPortPath);
int32_t LMB_GPS_ClosePort(void);
int32_t LMB_GPS_GetGNRMC(RMC_DATA* pstuRmcData);
int32_t LMB_GPS_GetGNGGA(GGA_DATA* pstuGgaData);
int32_t LMB_GPS_GetGNVTG(VTG_DATA* pstuVtgData);
int32_t LMB_GPS_GetGNGLL(GLL_DATA* pstuGllData);
int32_t LMB_GPS_GetGNGSA(GSA_DATA* pstuGsaData);
int32_t LMB_GPS_GetGPGSV(GSV_DATA* pstuGsvData);
/*** G-Sensor ***/
int32_t LMB_GSR_GetAxisData(AXIS_RAWDATA* pstuRawData);
int32_t LMB_GSR_GetAxisOffset(AXIS_RAWDATA* pstuRawData);
int32_t LMB_GSR_GetRegData(uint8_t ubReg, uint8_t* pubData);
int32_t LMB_GSR_SetRegData(uint8_t ubReg, uint8_t ubData);
/*** CAN Module ***/
int32_t LMB_CAN_OpenPort(int8_t* pbPortPath);
int32_t LMB_CAN_ClosePort(void);
int32_t LMB_CAN_ModuleInfo(int8_t* pstrModuleInfo);
int32_t LMB_CAN_DrivingInfo(DRIVING_INFO* pstuDrivingInfo);
int32_t LMB_CAN_EngineInfo(ENGINE_INFO* pstuEngineInfo);
int32_t LMB_CAN_VehicleID(int8_t* strVehicleID);
int32_t LMB_CAN_RawData(uint8_t* aryubRawData, uint8_t* pubAmount);
#if defined(__cplusplus)
}
#endif

#endif /* LMBINC_H_ */
