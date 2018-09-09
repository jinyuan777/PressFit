#define  IsPowderOnly 1//0:西醫    1:中醫
#define  IsMainMotorButton 1
#define  IsCuterOk 1 //0沒剪刀 1有剪刀
#define  NUM_STG_PAGES  20 // Total number of flash pages to be used for file storage
#define  MAX_BLOCK_SIZE_READ     64 // Use the maximum read block size of 64 bytes

#define  MAX_BLOCK_SIZE_WRITE    4096   // Use the maximum write block size of 4096 bytes
#define  FLASH_PAGE_SIZE         512    //  Size of each flash page
#define  BLOCKS_PR_PAGE  8  // 512/64 = 8
#define  MAX_NUM_BYTES   FLASH_PAGE_SIZE*NUM_STG_PAGES
#define  MAX_NUM_BLOCKS  BLOCKS_PR_PAGE*NUM_STG_PAGES

// Message Types
#define  READ_MSG          0x00  // Message types for communication with host
#define	 CommandACK		   0x01
#define	 WRITE_DAT		   0x07
// Machine States
#define  ST_WAIT_DEV    0x01  // Wait for application to open a device instance
#define  ST_IDLE_DEV    0x02  // Device is open, wait for Setup Message from host
#define  ST_RX_SETUP    0x04  // Received Setup Message, decode and wait for data
#define  ST_RX_FILE     0x08  // Receive file data from host
#define  ST_TX_FILE     0x10  // Transmit file data to host
#define  ST_TX_ACK      0x20  // Transmit ACK 0xFF back to host after every 8 packets
#define  ST_ERROR       0x80  // Error state

//unsigned char code * data PageIndices[20];
/*** [BEGIN] USB Descriptor Information [BEGIN] ***/
code unsigned int USB_VID = 0x10C4;
code unsigned int USB_PID = 0xEA61;

const char Ini_Location = -2;
code unsigned char USB_MfrStr[]=
{
   0x1A,
   0x03,
   'S',0,
   'i',0,
   'l',0,
   'i',0,
   'c',0,
   'o',0,
   'n',0,
   ' ',0,
   'L',0,
   'a',0,
   'b',0,
   's',0
};
code unsigned char USB_ProductStr[]=
{
   0x10,
   0x03,
   'U',0,
   'S',0,
   'B',0,
   ' ',0,
   'A',0,
   'P',0,
   'I',0
};
code unsigned char USB_SerialStr[]=
{
   0x0A,
   0x03,
   '1',0,
   '2',0,
   '3',0,
   '4',0
};

code unsigned char USB_MaxPower=15;
code unsigned char USB_PwAttributes=0x80;
code unsigned int USB_bcdDevice= 0x0100;
code unsigned char LengthFile[3] _at_ 0x2000;
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void  State_Machine(void);    // Determine new state and act on current state
void  Receive_Setup(void);    // Receive and decode setup packet from host
void  Receive_File(void);     // Receive file data from host
void  Page_Erase(unsigned char*);      // Erase a flash page
void  Page_Write(unsigned int);      // Write a flash page


//============================================
void PCA0_Init (void);
void SPI_Init();
void OSCILLATOR_Init (void);
void Powder(unsigned char type,unsigned char pwm);
//void _SET8255(unsigned char MOD);
//void PrintPack();
void _u14pc(unsigned char MOD);
void PresureUp();
void PresureDown();
void Delay(unsigned int DTime);
void i_Delay(unsigned int DTime);
void TPHControl();
void FiveStep();
void OneStep(unsigned int time);
void Timer0_Init(void);
void putchar (unsigned char cData);
char _getkey ();
void UART1_Init (void);
void _u14PAR();
void PowderLocate();
void PowderToRight();
void PowderToLeft();
void PowderNumber();
void PaperMotorForward();
void PaperMotorReverse();
void PaperMotorLocate();
void DisableAllDriver();
void Carbon(bit bSW);
void AccessCpu0(ECpu0Command Command);
void RxPackInfo();
void Cpu1CommandIdel();
unsigned char Cpu1Command();
void ClosePrinter();
void ADC0_Init ();
void EightStep();
void SpacePack();
void Packing();
void PackWithoutPrint();
void OnePack(unsigned char Type,bit IsPrint,bit IsCut);
void CheckPauseState(bit IsPrint);
void ShowPackType();
void PowderCoverOpen();
void CheckErrorCode(EErrorCode ECode);
void AutoClear();
void PackWithPrint();
void HeadWithPrint();
void Print(unsigned int Line,signed int Number);
void EMIF_Init();
void USBPacking();
void PackForUSB();
void USBPack(bit IsDrug,bit IsPrint);
void Shake(unsigned int n);
void DrugToRight();
void DrugCoverOpen();
void DrugLeaveRSW();
void ShowPackTypeUSB();
void Carbonini();
void _One(unsigned char Type,bit IsPrint);
void SpacePacking(byte SpaceNumber);
void MainMotorPress();

#define PrintTrue	1
#define PrintFalse	0
#define MotorStop 		1
#define MotorRight		2
#define MotorLeft		3
sfr16 SBRL1 = 0xB4;
sbit _RESET   	= P1 ^5;

sbit _RD   		= P1 ^6;
sbit _WR   		= P1 ^7;

sbit _A1       =  P3 ^1;
sbit _A0       =  P3 ^0;
sbit _Sel 	   =  P1 ^4; 
#define PowderPWM	0x40
#define TempPWM_MaxPower 	0xA0
#define TempPWM_Standby		0x70
#define TempPWM_Packing		0x80
#define TempPWM_OFF			0x50
#define Time_FiveStepDelay 		800	
//extern xdata unsigned char u14	;

idata signed char PowderLocationCounter;
idata unsigned char CursorLocation ;
idata unsigned char CursorLocationTemp ;		
idata unsigned char PowderCounter ;
idata unsigned char PowderCounterPS_LastState ;
idata unsigned char PowderMotorState ;
xdata unsigned char mem[2][520][55]; 
//xdata unsigned char men[520][55] 
//extern xdata unsigned char u15;

unsigned char u14;
unsigned char u15;
unsigned char u15pc;
unsigned char u14pc;
unsigned char u14pb;



bdata unsigned char u15_PA _at_ 0x21 ;
sbit DrugCounterPS 	= u15_PA ^0;
sbit PowderCounterPS = u15_PA ^1;
sbit DrugCoverPS 	= u15_PA ^2;
sbit PowderCoverPS	= u15_PA ^3;
sbit PaperMotorPS 	= u15_PA ^4;
sbit PresurePS1		= u15_PA ^5;
sbit PresurePS2 	= u15_PA ^6;
sbit NoUse		= u15_PA ^7;

bdata unsigned char u14_PA _at_ 0x20 ;
sbit RHall 		= u14_PA ^1;	// 新版 RHall = u14_PA ^1
sbit LHall 		= u14_PA ^0;
sbit PaperHall 	= u14_PA ^2;
sbit CarbonHall = u14_PA ^3;
sbit U14_5 = u14_PA ^5;
sbit U14_6 = u14_PA ^6;
sbit U14_7 = u14_PA ^7;

bdata unsigned char u15_PB _at_ 0x22 ;
sbit FG_Drug 		= u15_PB ^0;
sbit FG_Powder  	= u15_PB ^1;
sbit VacuumSW 		= u15_PB ^2;
sbit PaperEntrySW	= u15_PB ^3;
sbit PowderLSW 		= u15_PB ^4;
sbit PowderRSW		= u15_PB ^5;
sbit DrugLSW 		= u15_PB ^6;
sbit DrugRSW		= u15_PB ^7;

bdata unsigned char BitPackParameter;
sbit IsPrintPack= BitPackParameter ^0;
sbit IsAutoCut = BitPackParameter ^1;
sbit IsShake	= BitPackParameter ^2;
sbit IsName		= BitPackParameter ^3;
sbit IsDateTime	= BitPackParameter ^4;
sbit IsPatientID= BitPackParameter ^5;

#define LeftSW			 0
#define CursorLeftEntry  1
#define CursorLeftHall   2
#define CursorRightHall  3
#define CursorRightEntry 4
#define RightSW			 5

#define Time_USB				0xF000
#define CutterPosition			360//340

#define Time_DrugCoverPS		0x8000	// 0x5000
#define Time_PowderCoverPS		0x8000  // 0x5000

#define Time_DrugCounterPS		0x1000
#define Time_PowderCounterPS	0x1000

#define Time_PSDelay			100
#define Time_PaperMotorDelay 	800	

#define Time_DrugRSW			0xF000  // 0xF000
#define Time_DrugLeaveRSW		0x0400	// 0x0400

#define Time_DrugCoverPS		0x8000	// 0x5000
#define Time_PowderCoverPS		0x8000  // 0x5000

#define Time_PowderLSW			0xF000  // 0xF000
#define Time_PowderRSW			0xF000  // 0xF000

#define Time_PaperMotorPS		0x1000  // 0x1000
#define Time_PresurePS			0x5000



bit IsRxCommand=0;
bit IsPowderMove =0;
bit IsDrugMove = 0;
bit IsPackPause;
bit IsPacking	=0; 
sfr16 ADC0     = 0xbd;   
               
