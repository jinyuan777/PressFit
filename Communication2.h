typedef unsigned char byte ;
typedef unsigned int  word ;
typedef union
{
	word W ;
	byte B[2];
} TWord;

typedef enum
{
	Message_Heating 		=0,
	Message_PowderLocating	=1,
	Message_PackPowder		=2,
	Message_PackDrug		=3,
	Message_PackMix			=4,
	Message_PackSpace		=5,
	Message_Clearing		=6,
	Message_PackFinish		=7,
	Message_PackPause		=8,
	Message_PackCancel		=9,
	Message_SystemReady		=10,
	Message_Space			=11,

	Message_PowderLocationAlerm   =37,
	Message_PaperEntryAlerm		  =38,
	Message_CarbonEntryAlerm	  =39
} EMessage ;


typedef enum
{
	ErrorCode_NoError		=0,
	ErrorCode_ControlPanel	=1,
	ErrorCode_USBConnection	=2,
	ErrorCode_PaperMotor	=3,
	ErrorCode_Powder		=4,
	ErrorCode_Drug			=5,
	ErrorCode_PowderCover	=6,
	ErrorCode_DrugCover		=7,
	ErrorCode_PresureDown	=8,
	ErrorCode_PresureUp		=9,
	ErrorCode_Heater		=10,
	ErrorCode_HallSensor	=11,
	ErrorCode_Printer		=12
} EErrorCode ;


typedef enum
{
	UART_NoMark	=0xEA,
	UART_Mark	=0xEB,
} EUART ;



typedef enum
{
	KEY_Idel	=0,
	KEY_Print	=0x09,
	KEY_Function=0x0B,
	KEY_Enter	=0x0D,
	KEY_PgUp	=0x14,
	KEY_PgDown  =0x15,
	KEY_Esc		=0x1B,
	KEY_Right	=0x1C,
	KEY_Left	=0x1D,
	KEY_Up		=0x1E,
	KEY_Down	=0x1F,
	KEY_Del		=0x20,
	KEY_Stop	=0x23,
	KEY_Start	=0x2A,
	KEY_0		=0x30,
	KEY_1		=0x31,
	KEY_2		=0x32,
	KEY_3		=0x33,
	KEY_4		=0x34,
	KEY_5		=0x35,
	KEY_6		=0x36,
	KEY_7		=0x37,
	KEY_8		=0x38,
	KEY_9		=0x39
} EKEY ;



enum
{
	u14PC_CarbonStop  		= 0x00,
	u14PC_CarbonStart 		= 0x01,

	u14PC_PowderCoverStop  	= 0x02,
	u14PC_PowderCoverStart 	= 0x03,

	u14PC_DrugCoverStop 	= 0x04,
	u14PC_DrugCoverStart	= 0x05,

	u14PC_PresureStop 		= 0x06,
	u14PC_PresureStart 		= 0x07,
	u14PC_PowderRight		= 0x08,
	u14PC_PowderLeft		= 0x09,
	u14PC_PowderStop		= 0x0A,
	u14PC_PowderStart		= 0x0B,
	u14PC_DrugRight			= 0x0C,
	u14PC_DrugLeft			= 0x0D,
	u14PC_DrugStop			= 0x0E,
	u14PC_DrugStart			= 0x0F

} Eu14PCaa ;

enum u15PC
{
	u15PC_VacuumHighSpeedON 	= 0x00,
	u15PC_VacuumHighSpeedOFF  	= 0x01,
	u15PC_VacuumLowSpeedON		= 0x02,
	u15PC_VacuumLowSpeedOFF  	= 0x03,
	u15PC_HeaterControlL		= 0x04,
	u15PC_HeaterControlH		= 0x05,
	u15PC_SharkerOFF			= 0x06,
	u15PC_SharkerON				= 0x07,
	u15PC_PaperCKL				= 0x08,
	u15PC_PaperCKH				= 0x09,
	u15PC_PaperCCW				= 0x0A,
	u15PC_PaperCW				= 0x0B,
	u15PC_PaperResetL			= 0x0C,
	u15PC_PaperResetH			= 0x0D,
	u15PC_PaperDisable			= 0x0E,
	u15PC_PaperEnable			= 0x0F,
	u15PC_CarbonON				= 0x10,
	u15PC_CarbonOFF				= 0x11,
	u15PC_PaperPressON			= 0x12,
	u15PC_PaperPressOFF			= 0x13,

} Eu15PCaa ;

//--------------------------------------------------------
typedef enum 
{ 
    USBTxIndex_Mark, 			// 0
    USBTxIndex_Command,			// 1 
	USBTxIndex_Cpu1State,		// 2
    USBTxIndex_NowNumber,		// 3
    USBTxIndex_PowderLocation,	// 4
    USBTxIndex_Thermometer,		// 5
	USBTxIndex_ImageNumber,		// 6
    USBTxIndex_ErrorCode,		// 7
    USBTxIndex_Counter,			// 8
	USBTxIndex_Cpu0State		// 9
} EUSBTxIndex ;

typedef enum 
{ 
	USBRxIndex_Mark			=0,
	USBRxIndex_Cpu1Command 	=1,
	USBRxIndex_PackType		=2,
	USBRxIndex_TotalNumber 	=3,
	USBRxIndex_SpaceNumber 	=4,
	USBRxIndex_IsAutoFill 	=5,
	USBRxIndex_IsShake 		=6,
	USBRxIndex_HAddr		=7,
	USBRxIndex_LAddr		=8,
	USBRxIndex_Data			=9,	// 9~63

	USBRxIndex_PackNumber	=15,
	USBRxIndex_StartNumber	=16,

	USBRxIndex_Year		=17,
	USBRxIndex_Month	=18,
	USBRxIndex_Day		=19,
	USBRxIndex_IsPrintDate		=20,
	USBRxIndex_IsPrintNumber	=21

} USBRxIndex ;

//--------------------------------------------------------
typedef enum
{
	PackType_Powder,
	PackType_Drug,
	PackType_Mix,
	PackType_Space,
	PackType_Connection,
	PackType_Drug0,
	PackType_Drug1,
	PackType_Drug2,
	PackType_Drug3
} EPackType;

//--------------------------------------------------------
typedef enum
{
	ConnectionMode_0,
	ConnectionMode_1,
	ConnectionMode_2,
	ConnectionMode_3
} EConnectionMode;

//--------------------------------------------------------
typedef enum 					
{
	Cpu0State_Idel,				
	Cpu0State_Busy,					
	Cpu0State_SettingMode,			
	Cpu0State_PackingMode
} ECpu0State;



typedef enum 
{ 
    Cpu1State_Idel			=0,
	Cpu1State_Heating		=1,	
	Cpu1State_Packing		=2,	
	Cpu1State_USBPacking	=3,	
	Cpu1State_PackPause		=4,	
	Cpu1State_Clearing		=5,	
	Cpu1State_PowderLocating=6, 
    Cpu1State_Busy			=7,	
    Cpu1State_Error			=8,
	Cpu1State_Waiting		=9
} ECpu1State ;

// --------------------------------------------------------
typedef enum 					
{
	Cpu0Command_Idel,			
	Cpu0Command_TxPackInfo,		

	Cpu0Command_ShowInfo,			
	Cpu0Command_ShowPackInfo,		
	Cpu0Command_ShowSystemReady,
	Cpu0Command_ShowHeating,	
	Cpu0Command_ShowClearing,	
	Cpu0Command_ShowPowderLocating,
	Cpu0Command_ShowPackFinish, 
	Cpu0Command_ShowPaperEntry,	
	Cpu0Command_ShowPaperReady,	
	Cpu0Command_ShowCarbonEntry,
	Cpu0Command_ShowCarbonNoEntry,
	Cpu0Command_ShowPackPause,	
	Cpu0Command_ShowPackContinue,
	Cpu0Command_ShowPackStop,	
	Cpu0Command_ShowLeftDownGroupBox, 

	Cpu0Command_ShowPackPowder,
	Cpu0Command_ShowPackDrug,
	Cpu0Command_ShowPackMix,
	Cpu0Command_ShowPackSpace,

	Cpu0Command_ShowCPU1ErrorCode,
	Cpu0Command_SetDateTime,

	Cpu0Command_Reset			
} ECpu0Command ;


typedef enum 
{
	Cpu1Command_Idel,			
	Cpu1Command_RxPackInfo,

	Cpu1Command_USBPackStart,
	Cpu1Command_PackStart,	
	Cpu1Command_PackPause,	
	Cpu1Command_PackStop,	
	Cpu1Command_PackContinue,	
	Cpu1Command_PaperMotorLocate,
	Cpu1Command_PaperMotorForward,
	Cpu1Command_PaperMotorReverse,
	Cpu1Command_MototPressFit,		
	Cpu1Command_PowderLocate,	
	Cpu1Command_StopHeating,	
	
	Cpu1Command_MemroyWrite,	
	Cpu1Command_SetDateTime,
	Cpu1Command_Reset,			
	Cpu1Command_Again,
	Cpu1Command_Cut,
	Cpu1Command_PowderLeft
} ECpu1Command;

typedef enum 
{
	USBCommand_Idel,			
	USBCommand_Busy,			
	USBCommand_MemroyWrite,		
	USBCommand_ShowPaperEntry,
	USBCommand_ShowCarbonEntry,
	USBCommand_ShowCarbonNoEntry,
	USBCommand_ShowPackPause,
	USBCommand_ShowPackContinue,
	USBCommand_ShowPackStop,
	USBCommand_DrugsReady
} EUSBCommand;
// --------------------------------------------------------
typedef enum 
{
	VacuumType_Stop,
	VacuumType_HighSpeed,
	VacuumType_LowSpeed
} EVacuumType ;

typedef enum
{ 
	CommandType_Command, 
	CommandType_Memory 
} ECommandType ;



typedef enum
{
	ErrorUnit_PaperMotor   		=20,
	ErrorUnit_CarbonMotor  		=21,
	ErrorUnit_PowderMotor  		=22,
	ErrorUnit_DrugerMotor	   	=23,
	ErrorUnit_VCoverMotor  		=24,
	ErrorUnit_DrugerCoverMotor  =25,
	ErrorUnit_Printer	   		=29,
	ErrorUnit_Seal		   		=30,
	ErrorUnit_I2C		   		=31,
	ErrorUnit_UART		   		=32,
	ErrorUnit_USB		   		=33,
	ErrorUnit_Paramete	   		=34,

	ErrorUnit_PowderLocation   	=37,
	ErrorUnit_NoPaper	   		=38,
	ErrorUnit_NoCarbon	   		=39
} EErrorUnit ;

typedef enum
{
	ErrorType_MotorAlarm   =12,
	ErrorType_PSAlarm	   =13,
	ErrorType_RSWAlarm	   =14,
	ErrorType_LSWAlarm	   =15,
	ErrorType_HallSensor   =16,
	ErrorType_Alarm		   =17,
	ErrorType_Error		   =18,
	ErrorType_Warning	   =19
} EErrorType ;


typedef struct 
{
	byte Year;
	byte Day;
	byte Month;
	byte Date;
	byte Hour;
	byte Minute;
	byte Sec;	
} TDatetime;

typedef struct 
{
	ECpu0Command Command;			
	ECpu0State	 State;				
	EPackType 	 PackType;			
	byte		 SpaceNumber;		
	byte 	 	 TotalNumber;		
	byte 		 PrintTotalNumber;  
	byte	 	 BitPackParameter;	
	byte	 	 PrintItem[4];		
	byte 	 	 PrintItemNumber[4];
	byte	 	 ID[8];				
	TDatetime 	 DateTime ;			
} TCpu0 ;

typedef struct
{
	ECpu1Command 	Command_UART;		
	ECpu1Command 	Command_USB;
	ECpu1State 		State;	

	byte	NowNumber	;	
	byte  	PowderLocation;	
	byte  	Thermometer	;	
	byte    ImageNumber ;	

	TWord 	PowderStartPosition;
	TWord 	DrugStartPosition;	
	TWord 	InitTempTime;		
	TWord 	ShakeArray[6];		

	EVacuumType	VacuumType ;	

	TWord 	ForwardNumber;		
	TWord 	ReverseNumber;		
	byte  	PowderCoverOpenNumber;
	byte  	DrugCoverOpenNumber;
	byte  	HeaterPWM ;			
	
	EMessage		Message;		
} TCpu1;

typedef struct 
{
	EUSBCommand	Command;		
	EPackType 	PackType;		
	byte		SpaceNumber;	
	byte 		PackNumber;		
	byte 		StartNumber;	
	byte 	 	TotalNumber;	
    byte 		PgMode;		
	byte		IsShake;		

	byte		Year;			
	byte		Month;			
	byte 		Day;			
	byte        IsAutoCut;	
	byte        IsPrintNumber;	
} TUSB ;
typedef struct
{
	word		Left;
	word		Right;
	word		Length;
	byte		SpacePg;
} TSize;
typedef struct
{
	EUART  		UART_Mark; 	
	TCpu0 		Cpu0;		
	TCpu1 		Cpu1;				
	EErrorCode  ErrorCode; 	
	TSize		PgMode[3];
} TPacker;


