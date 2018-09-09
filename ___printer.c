#include <C8051F340.h>
#include <communication2.h>
//#include <CPU1.h>
#include <intrins.h>
//#include <a0.c>

//============================================
#include "USB_API.h"
#include "compiler_defs.h"
#define INTERRUPT_USBXpress 17
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

// Constants Definitions
#define  NUM_STG_PAGES  20 // Total number of flash pages to be used for file storage
#define  MAX_BLOCK_SIZE_READ     64 // Use the maximum read block size of 64 bytes

#define  MAX_BLOCK_SIZE_WRITE    4096   // Use the maximum write block size of 4096 bytes
#define  FLASH_PAGE_SIZE         512    //  Size of each flash page
#define  BLOCKS_PR_PAGE  8  // 512/64 = 8
#define  MAX_NUM_BYTES   FLASH_PAGE_SIZE*NUM_STG_PAGES
#define  MAX_NUM_BLOCKS  BLOCKS_PR_PAGE*NUM_STG_PAGES

// Message Types
#define  READ_MSG          0x00  // Message types for communication with host
#define  WRITE_MSG         0x01
#define  SIZE_MSG          0x02
#define  DELAYED_READ_MSG  0x05
#define  PrintStart        0x06
// Machine States
#define  ST_WAIT_DEV    0x01  // Wait for application to open a device instance
#define  ST_IDLE_DEV    0x02  // Device is open, wait for Setup Message from host
#define  ST_RX_SETUP    0x04  // Received Setup Message, decode and wait for data
#define  ST_RX_FILE     0x08  // Receive file data from host
#define  ST_TX_FILE     0x10  // Transmit file data to host
#define  ST_TX_ACK      0x20  // Transmit ACK 0xFF back to host after every 8 packets
#define  ST_ERROR       0x80  // Error state

// No such thing as a block of data anymore since it is variable between 1 and 1024
// So comment this out
typedef struct {        // Structure definition of a block of data
   char  Piece[MAX_BLOCK_SIZE_READ];
}  BLOCK;

typedef struct {        // Structure definition of a flash memory page
   char  FlashPage[FLASH_PAGE_SIZE];
}  PAGE;

xdata  BLOCK TempStorage[BLOCKS_PR_PAGE];
//SEGMENT_VARIABLE(TempStorage[BLOCKS_PR_PAGE], BLOCK, SEG_XDATA); //Temporary storage of between flash writes
//data code unsigned  char* 
//SEGMENT_VARIABLE_SEGMENT_POINTER(PageIndices[20], U8, SEG_CODE, SEG_DATA);
//# define SEGMENT_VARIABLE(name, vartype, locsegment)  locsegment vartype name
unsigned char code * data PageIndices[20];
//# define SEGMENT_VARIABLE_SEGMENT_POINTER(name, vartype, targsegment, locsegment) vartype targsegment * locsegment name

SEGMENT_VARIABLE(BytesToRead, U16, SEG_DATA); // Total number of bytes to read from host
SEGMENT_VARIABLE(WriteStageLength, U16, SEG_DATA); //  Current write transfer stage length
SEGMENT_VARIABLE(ReadStageLength, U16, SEG_DATA);  //  Current read transfer stage length
SEGMENT_VARIABLE(Buffer[3], U8, SEG_DATA);   // Buffer for Setup messages
SEGMENT_VARIABLE(NumBytes, U16, SEG_DATA);   // Number of Blocks for this transfer
SEGMENT_VARIABLE(NumBlocks, U8, SEG_DATA);
SEGMENT_VARIABLE(BytesRead, U16, SEG_DATA);  // Number of Bytes Read
SEGMENT_VARIABLE(M_State, U8, SEG_DATA);     // Current Machine State
SEGMENT_VARIABLE(BytesWrote, U16, SEG_DATA); // Number of Bytes Written
SEGMENT_VARIABLE(BlockIndex, U8, SEG_DATA);  // Index of Current Block in Page
SEGMENT_VARIABLE(PageIndex, U8, SEG_DATA);   // Index of Current Page in File
SEGMENT_VARIABLE(BlocksWrote, U8, SEG_DATA); // Total Number of Blocks Written
#if defined __C51__
   SEGMENT_VARIABLE(ReadIndex, U8*, SEG_DATA);
#elif defined SDCC
   SEGMENT_VARIABLE_SEGMENT_POINTER(ReadIndex, U8, SEG_CODE, SEG_DATA);
#endif
SEGMENT_VARIABLE(BytesToWrite, U16, SEG_DATA);

/*** [BEGIN] USB Descriptor Information [BEGIN] ***/
code unsigned int USB_VID = 0x10C4;
code unsigned int USB_PID = 0xEA61;

//SEGMENT_VARIABLE(USB_VID, U16, SEG_CODE) = 0x10C4;
//SEGMENT_VARIABLE(USB_PID, U16, SEG_CODE) = 0xEA61;
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


//LOCATED_VARIABLE_NO_INIT(LengthFile[3], U8, SEG_CODE, 0x2000);
   // {Length(Low Byte), Length(High Byte), Number of Blocks}
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
//void   Suspend_Device(void);      //  Place the device in suspend mode
void  Page_Erase(unsigned char*);      // Erase a flash page
void  Page_Write(unsigned char*);      // Write a flash page


//============================================
void PCA0_Init (void);
void SPI_Init();
void OSCILLATOR_Init (void);
void Powder(unsigned char type,unsigned char pwm);
void _SET8255(unsigned char MOD);

//extern xdata unsigned char mem[520][55] ;
	
void _OnePack();
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
void PaperMotorForward();
void PaperMotorReverse();
void DisableAllDriver();
void Carbon(bit bSW);
#define MotorStop 		1
#define MotorRight		2
#define MotorLeft		3
sfr16 SBRL1 = 0xB4;
sbit _RESET   	= P1 ^5;

sbit _RD   		= P1 ^6;
sbit _WR   		= P1 ^7;

sbit _A1       =  P3 ^1;
sbit _A0       =  P3 ^0;

#define PowderPWM	0x60
#define TempPWM_MaxPower 	0xF0
#define TempPWM_Standby		0x58
#define TempPWM_Packing		0x60
#define TempPWM_OFF			0x58

//extern xdata unsigned char u14	;
unsigned char u14	;
idata signed char PowderLocationCounter;
idata unsigned char CursorLocation ;	
idata unsigned char PowderCounter ;
idata unsigned char PowderCounterPS_LastState ;
idata unsigned char PowderMotorState ;

//extern xdata unsigned char mem[520][55] ;	
extern xdata unsigned char u15;
unsigned char u15pc;
unsigned char u14pc;
unsigned char u14pb;
bdata unsigned char u15_PA _at_ 0x21 ;
sbit DrugCounterPS 	= u15_PA ^0;
sbit PowderCounterPS= u15_PA ^1;
sbit DrugCoverPS 	= u15_PA ^2;
sbit PowderCoverPS	= u15_PA ^3;
sbit PaperMotorPS 	= u15_PA ^4;
sbit PresurePS1		= u15_PA ^5;
sbit PresurePS2 	= u15_PA ^6;
sbit NoUsePS		= u15_PA ^7;

bdata unsigned char u14_PA _at_ 0x20 ;
sbit RHall 		= u14_PA ^1;	// 新版 RHall = u14_PA ^1
sbit LHall 		= u14_PA ^0;
sbit PaperHall 	= u14_PA ^2;
sbit CarbonHall = u14_PA ^3;

bdata unsigned char u15_PB _at_ 0x22 ;
sbit FG_Drug 		= u15_PB ^0;
sbit FG_Powder  	= u15_PB ^1;
sbit VacuumSW 		= u15_PB ^2;
sbit PaperEntrySW	= u15_PB ^3;
sbit PowderLSW 		= u15_PB ^4;
sbit PowderRSW		= u15_PB ^5;
sbit DrugLSW 		= u15_PB ^6;
sbit DrugRSW		= u15_PB ^7;

#define LeftSW			 0
#define CursorLeftEntry  1
#define CursorLeftHall   2
#define CursorRightHall  3
#define CursorRightEntry 4
#define RightSW			 5
//-----------------------------------------------------------------------------
// UART1 Global Variables
//-----------------------------------------------------------------------------
/*
#define UART_BUFFERSIZE 64
unsigned char UART_Buffer[UART_BUFFERSIZE];
unsigned char UART_Buffer_Size = 0;
unsigned char UART_Input_First = 0;
unsigned char UART_Output_First = 0;
unsigned char TX_Ready =1;
static char Byte;*/

//-----------------------------------------------------------------------------
// 主程式
//-----------------------------------------------------------------------------
extern xdata TPacker Packer ;
void Cuter(void){
		P2 = P2 & 0x6F;
		P4 = 0xFF;
		_RESET = 0;
		_RD = 1;
		_WR	= 0;
		_A1=0;
		_A0=1;	
}

void ini_u14(){
	P2=0x6F;
	Delay(20);
//	_RESET = 1;
//	_RESET = 0;
	P4=0x90;
	_A1=1;_A0=1;
	_RD=1;
	_WR=0;
	Delay(10);
	_WR=1;
	Delay(20);
}

void ini_u15(){
	P2=0x7F;
	_RESET = 1;
	_RESET = 0;
	P4=0x92;
	_A1=1;_A0=1;
	_RD=1;
	_WR=0;
	Delay(10);
	_WR=1;
	Delay(20);
}

unsigned char _PowderMax=30;
unsigned char _PowderPWM=25;

void Powder(unsigned char type,unsigned char pwm){
	PCA0CPH0=pwm;
	if (type == MotorStop)
	{
		u14 = u14PC_PowderStop;
	}
	else if (type == MotorLeft)
	{
		u14 = u14PC_PowderLeft;
	}
	else if (type ==MotorRight)
	{
		u14 = u14PC_PowderRight;
    }
}
void _u14pb(unsigned char MOD){
	P2=0x6F;
	P4=MOD;
	_RESET = 0;
	_A1=0;_A0=1;
	_RD=1;
	_WR=0;
	_WR=1;
}
void _u14pc(unsigned char MOD){
	P2=0x6F;
	P4=MOD;
	_RESET = 0;
	_A1=1;_A0=0;
	_RD=1;
	_WR=0;
	_WR=1;	
}
void _u15pc(unsigned char MOD){
	P2=0x7F;
	P4=MOD;
	_RESET = 0;
	_RD=1;
	_A1=1;_A0=0;
	_WR=0;
	_WR=1;			
}
//-----------------------------------------------------------------------------
void _Print(unsigned int Line)
{
	unsigned char i,d;

	#define Right 	00
	#define Left  	520
	#define Bottom  20
	//temp = cA;
	unsigned char tmpEIE1;
	unsigned char tmpEIE2;

	unsigned int Shift = 0 ;

	tmpEIE1 = EIE1;
	tmpEIE2 = EIE2;
	EA =0;
	EIE1 = 0;
	EIE2 = 0;	
//	u15 = u15PC_PaperCKH;
	_u15pc(0xF0);	
	if (Line < Right || Line >= Right +520)
	{
		for (i=1;i<=80;i++)
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}
	}
	else 
	{
		for (i=1;i<=80;i++)
		{
			while(!TXBMT);
			SPIF=0;		
			if (i<=Bottom || i>=Bottom +55 ) 
				SPI0DAT = 0x00;
			else{   					
			//	SPI0DAT = mem[Line-Right][i-Bottom];
			//	if(Line>=60&&Line<460&&i>=35&&i<42)
			//		SPI0DAT = AA[Line-60][i-35];
			//	else
				SPI0DAT = 0xF0;	
			
			}
		}
	}

	d=100; while(d--);

	TPHControl();

	EA=1;
	EIE1 = tmpEIE1 ;
	EIE2 = tmpEIE2 ;
	FiveStep();
}
void _u15PAR(){
	P2=0x7F;
	P4|=0x60;
	_WR=1;_RD=0;
	_A1=0;_A0=0;
	u15_PA = P4;	
}
void _u15PBR(){
	P2=0x7F;
	P4|=0xFC;
	_WR=1;_RD=0;
	_A1=0;_A0=1;
	u15_PB = P4;	
}
void _u14PAR(){
	P2=0x6F;
	P4|=0x0F;
	_WR=1;_RD=0;
	_A1=0;_A0=0;
	u14_PA = P4;	
}

//-----------------------------------------------------------------------------
void Port_Ini(){
    P0MDOUT   = 0x88;
    P1MDOUT   = 0xE0;
    P2MDOUT   = 0xFF;
    P3MDOUT   = 0x03;
    P0SKIP    = 0x71;
    XBR0      = 0x02;
    XBR1      = 0x43;
    XBR2      = 0x01;
}


void Interrupts_Init()
{
    EIE2      = 0x02;
    IE        = 0x80;
}
unsigned char Params=0;  
unsigned char tempIO=0x00;
void EEPROM_ByteWrite(unsigned char bank,unsigned char HighAddr,unsigned char LowAddr, unsigned char dat){
	unsigned char i,j;
	unsigned int Addr;
	EA=0;
	for(i=0;i<400;i++){
		for(j=0;j<6;j++){
			Addr=i*j;
			P3 = Addr&0x00FF;
			P2 = Addr>>8;
			P2 |=0x80;
			P4 = dat;
			_WR=0;
			_WR=1;
		}
	}
	EA=1;
}
void main()
{   
	unsigned int Addr=0x0000;
	unsigned char j=0,i=0;
	unsigned char numbers[]={0x17,0x07,0x03,0x10},ReadNumbers[]={0x00,0x00,0x00,0x00};
	//==============================
    PCA0MD &= ~0x40; 
    PageIndices[0]  = 0x2200;
    for (j=1; j<20; j++)
    {
       PageIndices[j] = PageIndices[j-1] + 0x200;
    }
    USB_Clock_Start();            // Init USB clock *before* calling USB_Init
    USB_Init(USB_VID,USB_PID,USB_MfrStr,USB_ProductStr,USB_SerialStr,USB_MaxPower,USB_PwAttributes,USB_bcdDevice);	
	//==============================
	Addr = 0x0000;

//	UART1_Init();
	SPI_Init();
	Port_Ini();
	DisableAllDriver();
    OSCILLATOR_Init ();                 // Initialize oscillator
    PCA0_Init ();                       // Initialize PCA0
	ini_u15();
//	Interrupts_Init();
	ini_u14();
	Timer0_Init();
    USB_Int_Enable();             // Enable USB_API Interrupts
	DisableAllDriver();
	u14pc=0x00;u15pc=0x40;
	u14pb=0xFF;
	_u14pb(u14pb);
//	PresureDown();
/*
	EA=0;
	for(i=0;i<4;i++){
		P3 = 0x00+i;
		P2 = 0x00;
		P2 |=0x80;	
		P4 = numbers[i];
		_WR=0;
		_WR=1;
	}
	P4=0xff;
	for(i=0;i<4;i++){
		P3 = 0x00+i;
		P2 = 0x00;
		P2 |=0x80;
		_RD=0;
		ReadNumbers[i]=P4;
		_RD=1;
	}
	EA=1;*/
	while(1){
		if(Packer.Cpu1.Command_USB == Cpu1Command_PackStart){
			PresureDown();
		}
		//	_OnePack();	

	}
}
//===============================================
void  Page_Erase (unsigned char*  Page_Address)
{
   unsigned char  EA_Save;               // Used to save state of global interrupt enable
   unsigned char  xdata *pwrite;     // xdata pointer used to generate movx intruction

   EA_Save  =  EA;            // Save current EA
   EA =  0;                   // Turn off interrupts
   pwrite   =  (unsigned char xdata *)(Page_Address); // Set write pointer to Page_Address
   PSCTL =  0x03;             // Enable flash erase and writes

   FLKEY =  0xA5;             // Write flash key sequence to FLKEY
   FLKEY =  0xF1;
   *pwrite  =  0x00;          // Erase flash page using a write command

   PSCTL =  0x00;             // Disable flash erase and writes
   EA =  EA_Save;             // Restore state of EA
}
void  Page_Write (unsigned char*  PageAddress)
{
   unsigned char  EA_Save;             // Used to save state of global interrupt enable
   unsigned char  xdata *pwrite;   // Write Pointer
   unsigned char  xdata *pread;    // Read Pointer
   unsigned int  x;                  // Counter for 0-512 bytes

   pread =  (unsigned char xdata *)(TempStorage);
   EA_Save  =  EA;            // Save EA
   EA =  0;                   // Turn off interrupts
   pwrite   =  (unsigned char xdata *)(PageAddress);
   PSCTL =  0x01;             // Enable flash writes
   for(x = 0;  x<FLASH_PAGE_SIZE;   x++) // Write 512 bytes
   {
      FLKEY =  0xA5;          // Write flash key sequence
      FLKEY =  0xF1;
      *pwrite  =  *pread;     // Write data byte to flash

      pread++;                // Increment pointers
      pwrite++;
   }
   PSCTL =  0x00;             // Disable flash writes
   EA =  EA_Save;             // Restore EA
}

void  State_Machine(void)
{
   switch   (M_State)
   {
      case  ST_RX_SETUP:
         Receive_Setup();           // Receive and decode host Setup Message
         break;
      case  ST_RX_FILE:
         Receive_File();            // Receive File data from host
         break;
      case  ST_TX_ACK:
         M_State =   ST_RX_FILE;    // Ack Transmit complete, continue RX data
         break;
      case  ST_TX_FILE:             // Send file data to host
         WriteStageLength = ((BytesToWrite - BytesWrote) > MAX_BLOCK_SIZE_WRITE)? MAX_BLOCK_SIZE_WRITE:(BytesToWrite - BytesWrote);
         BytesWrote  += Block_Write((unsigned char*)(ReadIndex), WriteStageLength);
         ReadIndex += WriteStageLength;

         //if ((BlocksWrote%8) == 0)  Led2 = !Led2;
         //if (BytesWrote == NumBytes)   Led2 = 0;
         break;
      default:
         break;
   }
}


   // ISR for USB_API, run when API interrupts are enabled, and an interrupt is received
void USB_API_TEST_ISR(void)interrupt 	INTERRUPT_USBXpress
{
   unsigned char  INTVAL   =  Get_Interrupt_Source();  // Determine type of API interrupts
   if (INTVAL  &  USB_RESET)                // Bus Reset Event, go to Wait State
   {
      M_State  =  ST_WAIT_DEV;
   }

   if (INTVAL  &  DEVICE_OPEN)            // Device opened on host, go to Idle
   {
      M_State  =  ST_IDLE_DEV;
   }

   if (INTVAL  &  TX_COMPLETE)
   {
      if (M_State == ST_RX_FILE)          // Ack Transmit complete, go to RX state
      {
         M_State  =  (ST_TX_ACK);
      }
      if (M_State == ST_TX_FILE)          // File block transmit complete, go to TX state
      {
         M_State  =  (BytesWrote == BytesToWrite) ? ST_IDLE_DEV :ST_TX_FILE;  // Go to Idle when done
      }
   }
   if (INTVAL  &  RX_COMPLETE)            // RX Complete, go to RX Setup or RX file state
   {
      M_State  =  (M_State == ST_IDLE_DEV) ? ST_RX_SETUP : ST_RX_FILE;
   }
   if (INTVAL  &  DEVICE_CLOSE)           // Device closed, wait for re-open
   {
      M_State  =  ST_WAIT_DEV;
   }
   if (INTVAL  &  FIFO_PURGE)             // Fifo purged, go to Idle State
   {
      M_State  =  ST_IDLE_DEV;
   }

   State_Machine();                       // Call state machine routine
}

void  Receive_Setup (void)
{
   #if defined SDCC
      SEGMENT_VARIABLE_SEGMENT_POINTER(temp_address, U8, SEG_DATA, SEG_DATA) = 0x2000;
   #endif


   BytesRead   =  Block_Read(&Buffer,  3);      // Read Setup Message


   if (Buffer[0]  == READ_MSG)         // Check See if Read File Setup
   {
      PageIndex   =  0;                // Reset Index
      NumBlocks   =  LengthFile[2];    // Read NumBlocks from flash stg
      NumBlocks   = (NumBlocks > MAX_NUM_BLOCKS)?  MAX_NUM_BLOCKS:   NumBlocks;
                                          // only write as many bytes
                                          // as we have space available
      Buffer[0]   =  SIZE_MSG;            // Send host size of transfer message
      Buffer[1]   =  LengthFile[1];
      Buffer[2]   =  LengthFile[0];
      BytesToWrite   =  Buffer[1]   +  256*Buffer[2];
      BytesWrote  =  Block_Write((unsigned char*)&Buffer,   3);
      M_State  =  ST_TX_FILE;             // Go to TX data state
      BytesWrote  =  0;

      ReadIndex   =  PageIndices[0];

   }/*
   else if(Buffer[0] == PrintStart){
   		M_State = ST_IDLE_DEV;
		//_OnePack();
		Packer.Cpu1.Command_USB = Cpu1Command_PackStart;
   }*/
   else  // Otherwise assume Write Setup Packet
   {
      BytesToRead =  Buffer[1]   +  256*Buffer[2];
      NumBlocks   =  (unsigned char)(BytesToRead/MAX_BLOCK_SIZE_READ);  // Find NumBlocks

      if (BytesToRead > MAX_NUM_BYTES)                // State Error if transfer too big
      {
         M_State = ST_ERROR;
		 	
      }
      else
      {

         if (BytesToRead%MAX_BLOCK_SIZE_READ)   NumBlocks++;   // Increment NumBlocks for last partial block

         TempStorage->Piece[0]   =  Buffer[2];
         TempStorage->Piece[1]   =  Buffer[1];
         TempStorage->Piece[2]   =  NumBlocks;

         // Write Values to Flash

         Page_Erase(0x2000);                       // Store file data to flash
         Page_Write(0x2000);


         PageIndex   =  0;                         // Reset Index
         BlockIndex  =  0;
         BytesRead   =  0;
         M_State = ST_RX_FILE;                     // Go to RX data state
      }
   }
}

void  Receive_File(void)
{
   ReadStageLength = ((BytesToRead - BytesRead) > MAX_BLOCK_SIZE_READ)? MAX_BLOCK_SIZE_READ:(BytesToRead - BytesRead);

   BytesRead   += Block_Read((unsigned char*)(&TempStorage[BlockIndex]), ReadStageLength);   // Read Block

   BlockIndex++;
   // If device has received as many bytes as fit on one FLASH page, disable interrupts,
   // write page to flash, reset packet index, enable interrupts
   // Send handshake packet 0xFF to host after FLASH write
   if ((BlockIndex   == 8) || (BytesRead  == BytesToRead))	                                                                                                                                                                                                           
   {
      Page_Erase((unsigned char*)(PageIndices[PageIndex]));
      Page_Write((unsigned char*)(PageIndices[PageIndex]));
      PageIndex++;
      BlockIndex  =  0;
      Buffer[0]   =  0xFF;
      Block_Write(Buffer,  1);         // Send handshake Acknowledge to host
   }

   // Go to Idle state if last packet has been received
   if (BytesRead  == BytesToRead)   {M_State =  ST_IDLE_DEV;   }
}

//===============================================
//-----------------------------------------------------------------------------
void PowderToLeft()
{
	Powder(MotorLeft,PowderPWM);	
//	errorTime = Time_PowderLSW ;
	while (PowderLSW)
	{
		Delay(1000);
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_Powder);
	}
	Powder(MotorStop,0xFF);
	Delay(1000);
}
//-----------------------------------------------------------------------------
void PowderToRight()
{
	Powder(MotorRight,PowderPWM);	
//	errorTime = Time_PowderLSW ;

	while (PowderRSW)
	{
		Delay(1000);
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_Powder);
	}
	Powder(MotorStop,0xFF);
	Delay(1000);
}
//-----------------------------------------------------------------------------
void PowderLocate()
{
//	if ( IsFR ==1)
//	{
//		CloseLeave();
//	}
	PowderToLeft();

	while (1)	
	{
		unsigned long SWWaitTime = 400; 	

		if (PowderLSW ==0)					
		{
			unsigned char s=0;
			Powder(MotorStop,0xFF);
			Delay(1000);

			// 碰到左SW,往右移 5格 停在透光
			CursorLocation = CursorRightEntry;
//			Powder(MotorRight,PowderPWM);
//			while(s < 3)
//			{
//				while(PowderCounterPS ==1) ;
//				s ++;
//				if (s >=3) break;
//				while(PowderCounterPS ==0) ;
//				s ++;
//			}
//			Powder(MotorStop,0xFF);
//			Delay(1000);
			PowderLocationCounter =2;
			PowderCounterPS_LastState = PowderCounterPS ;

			Powder(MotorRight,PowderPWM);
			Delay(SWWaitTime);

//			CheckErrorCode(ErrorCode_HallSensor);
		}

		if (PowderRSW ==0)					
		{
			Powder(MotorStop,0xFF);
			Delay(1000);

			PowderLocationCounter = 92;

			CursorLocation = CursorLeftEntry ; 	
	
			Powder(MotorLeft,PowderPWM);
			Delay(SWWaitTime);
//			CheckErrorCode(ErrorCode_HallSensor);
		}
		else if ( CursorLocation == CursorLeftEntry)	//1
		{
			Powder(MotorLeft,PowderPWM);
		}
		else if ( CursorLocation == CursorRightEntry)	//4
		{
			Powder(MotorRight,PowderPWM);
		}
		else if ( CursorLocation == CursorLeftHall )	//2
		{
			unsigned long l=0x50000; 			
			Powder(MotorStop,0xFF);
			Delay(500);

			Powder(MotorRight,0xA0);			
//			while (PowderCounterPS == 0) ;
//			while (PowderCounterPS == 1) ;
			while (CursorLocation != CursorRightHall)
			{
				if (l-- ==0) break;
			}
			if (CursorLocation == CursorRightHall)
				break;
		}
			
	}
	Powder(MotorStop ,0xFF);
	PowderCounterPS_LastState = PowderCounterPS ;
}

//-----------------------------------------------------------------------------
void Carbon(bit bSW){
	PCA0CPH0=0x60;
	if (bSW)
	{
		u14 = u14PC_CarbonStart ;
	}
	else
	{
		u14 = u14PC_CarbonStop ;
	}
}
//-----------------------------------------------------------------------------
void PresureDown()
{
	u14 =  u14PC_PresureStart;
	do 
	{
		Delay(1000);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS2==1) ;
//	errorTime = Time_PresurePS ;

	do 
	{
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS2==0) ;

	u14 = u14PC_PresureStop;
	Delay(800);
}
//-----------------------------------------------------------------------------
void PresureUp()
{
	u14 =  u14PC_PresureStart;

	do 
	{
		Delay(1000);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS1==1);//while(P4==0x20) ;
//	errorTime = Time_PresurePS ;

	do 
	{
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS1==0);//while(P4==0x00) ;

	u14 = u14PC_PresureStop;
	Delay(800);
}

//-----------------------------------------------------------------------------
void _TPHControl()
{
	unsigned char d;
	u14pb = 0xEF;	
	d=100;while(d--);
	//u14pb = 0xFF;	
	_u14pb(0xFF);

	d = 160;	while(d--);	
	// step 1	

	u15 = u15PC_PaperCKL;	
		
	u14pb = 0xFC;

	d=2000;while(d--); 

	u14pb = 0xFF;	

	d = 10;	while(d--);	
	// step 2
	u15 = u15PC_PaperCKH;		

	u14pb = 0xF3 ;		

	d=2000;while(d--);
  
	u14pb = 0xFF ;
}
void TPHControl()
{
	unsigned char d;
	_u14pb(0xEF);	//LATCH
	d=100;while(d--);
	_u14pb(0xFF);	

	d = 160;	while(d--);	
	// step 1	

	_u15pc(0xE0);	//PAPERMOTOR_LOW
		
	_u14pb(0xFC);//STROBE1,2
	
	Delay(25);

	_u14pb(0xFF);	

	d = 10;	while(d--);	
	// step 2
	_u15pc(0xF0);	//PAPERMOTOR_HIGHT	

	_u14pb(0xF3);//STROBE3,4	

	Delay(25);
  
	_u14pb(0xFF);
}
//-----------------------------------------------------------------------------
void FiveStep()
{
	unsigned int d;
	u15 = u15PC_PaperCKL;		
	d = 700;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = 700;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 700;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = 700;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 700;	while(d--);
}
//-----------------------------------------------------------------------------
void _OnePack() 
{
	unsigned int n;	
	PresureUp();
	Carbon(1);
	Delay(1);
	u15 = u15PC_PaperCW ;
	Delay(1);
	u15 = u15PC_PaperEnable;
	Delay(1);
	PCA0CPH0 = 0x60;
	for (n=0;n< 640 ;n++)
	{
		_Print(n);
	}
	Carbon(0);
	Delay(1);
	PresureDown();
	u15 = u15PC_PaperDisable;
//	ClosePrinter();
	
}
//-----------------------------------------------------------------------------
void PaperMotorForward()
{
	unsigned int i;
	u15 = u15PC_PaperCW ;
	Delay(1);
	u15 = u15PC_PaperEnable;
	Delay(1);
	for (i=0 ; i < 80 ; i++)
	{
		OneStep(800);	
	}

	u15 = u15PC_PaperDisable;
}
//-----------------------------------------------------------------------------
void PaperMotorReverse()
{
	unsigned int i;
	u15 = u15PC_PaperCCW ;
	Delay(1);
	u15 = u15PC_PaperEnable;
	Delay(1);
	for (i=0 ; i < 80 ; i++)
	{
		OneStep(800);	
	}
	u15 = u15PC_PaperDisable;
}
//-----------------------------------------------------------------------------
void Delay(unsigned int DTime)
{
//	i=i<<1;
	while(DTime--)
	{
		unsigned char DTime2=100;	// 1mSec
		while(DTime2--);

	}
}
void i_Delay(unsigned int DTime)
{
//	i=i<<1;
	while(DTime--)
	{
		unsigned char DTime2=100;	// 1mSec
		while(DTime2--);

	}
}

//-----------------------------------------------------------------------------
void PCA0_Init (void)
{
   // Configure PCA time base; overflow interrupt disabled
   PCA0CN = 0x40;                      // Stop counter; clear all flags
   PCA0MD = 0x00;                      // Use SYSCLK as time base

   PCA0CPM0 = 0x42;                    // Module 0 = 8-bit PWM mode

   // Configure initial PWM duty cycle = 50%
   PCA0CPH0 = 0xFF;

   // Start PCA counter
   CR = 1;
}
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its maximum frequency

   CLKSEL = 0x00;                      // Select the internal osc. as
                                       // the SYSCLK source
   OSCICL = 0x10;
   CLKSEL |= 0x03;
   RSTSRC   |= 0x02;
}
//-----------------------------------------------------------------------------
void OneStep(unsigned int time)
{
	unsigned int d ;

	u15 = u15PC_PaperCKH;		
	d = time;	while(d--);

	u15 = u15PC_PaperCKL;		
	d = time;	while(d--);

	u15 = u15PC_PaperCKH;		
	d = time;	while(d--);

	u15 = u15PC_PaperCKL;	
	d = time;	while(d--);
	
	u15 = u15PC_PaperCKH;	
	d = time;	while(d--);

	u15 = u15PC_PaperCKL;	
	d = time;	while(d--);
	
	u15 = u15PC_PaperCKH;	
	d = time;	while(d--);

	u15 = u15PC_PaperCKL;		
	d = time-40;	while(d--);
}

//-----------------------------------------------------------------------------
void SPI_Init()
{
    SPI0CFG   = 0x40;
    SPI0CN    = 0x01;
 //   SPI0CKR   = 0x4A;
 	SPI0CKR   = 0x07;
}
void Timer0_Init(void)
{
   TH0 = -31.25;           // Init Timer0 High register
   TL0 = TH0;                          // Set the intial Timer0 value

   TMOD = 0x02;                        // Timer0 in 8-bit reload mode
   CKCON = 0x00;                       // Timer0 uses a 1:48 prescaler
   ET0=1;                              // Timer0 interrupt enabled
   TCON = 0x10;                        // Timer0 ON
}
//char sMotor=0x00;
void Timer0_ISR (void) interrupt 1
{
	TR0=0;
	switch(u14){
		case u14PC_CarbonStop:
			u14pc &= 0xFE;
			break;
		case u14PC_CarbonStart:
			u14pc |= 0x01;
			break;
		case u14PC_PowderCoverStop:
			u14pc &= 0xFD;
			break;
		case u14PC_PowderCoverStart:
			u14pc |= 0x02;
			break;
		case u14PC_DrugCoverStop:
			u14pc &= 0xFB;
			break;
		case u14PC_DrugCoverStart:
			u14pc |= 0x04;
			break;
		case u14PC_PresureStop:
			u14pc &= 0xF7;
			break;
		case u14PC_PresureStart:
			u14pc |= 0x08;
			break;
		case u14PC_PowderRight:
			PCA0CPH0=0x60;
			u14pc &= 0xEF;
			break;
		case u14PC_PowderLeft:
			PCA0CPH0=0x60;
			u14pc |= 0x10;
			break;
		case u14PC_PowderStop:
			u14pc &= 0xDF;
			break;
		case u14PC_PowderStart:
			u14pc |= 0x20;
			break;
		case u14PC_DrugRight:
			u14pc &= 0xBF;
			break;
		case u14PC_DrugLeft:
			u14pc |= 0x40;
			break;
		case u14PC_DrugStop:
			u14pc &= 0x7F;
			break;
		case u14PC_DrugStart:
			u14pc |= 0x80;
			break;
	}
	switch(u15){
		case u15PC_PaperCKL:
			u15pc &= 0xEF;
			break;
		case u15PC_PaperCKH:
			u15pc |= 0x10;
			break;
		case u15PC_PaperCCW:
			u15pc &= 0xDF;
			break;
		case u15PC_PaperCW:
			u15pc |= 0x20;
			break;
		case u15PC_PaperDisable:
			u15pc &= 0x7F;
			break;
		case u15PC_PaperEnable:
			u15pc |= 0xC0;
			break;
		case u15PC_PaperResetL:
			u15pc &= 0xBF;
			break;
		case u15PC_PaperResetH:
			u15pc |= 40;
			break;
	}
//	_u14pb(u14pb);
	_u14pc(u14pc);
	_u15pc(u15pc);

	_u14PAR();
	if(!LHall)
		CursorLocation = CursorLeftHall;
	else if(!RHall)
		CursorLocation = CursorRightHall;
	_u15PAR();
	if(PowderCounterPS_LastState == !PowderCounterPS){
		if(CursorLocation == CursorRightEntry&&PowderCounterPS==0)
			PowderLocationCounter ++;
		else if(CursorLocation == CursorLeftEntry&&PowderCounterPS==0)
			PowderLocationCounter --;
		PowderCounterPS_LastState = PowderCounterPS;
	}		
	_u15PBR();
	TR0=1;
}

void UART1_Init (void){
	SBRLL1    = 0x3C;
    SBRLH1    = 0xF6;
    SCON1     = 0x10;		//埰勍UART1諉彶
    SMOD1     = 0x0C;
    SBCON1    = 0x43;  
}
//-----------------------------------------------------------------------------

void putchar(unsigned char cData)
{
	unsigned int i=10000;
	while( (SCON1 & 0x20) ==0) ;
	SBUF1 = cData;
	while(i--);	
}

//-----------------------------------------------------------------------------
char _getkey ()  {
    char c;
    while (!(SCON1 & 0x01));           // wait until UART1 receives a character
    c = SBUF1;                         // save character to local variable
    SCON1 &= ~0x01;                    // clear UART1 receive interrupt flag
    return (c);                        // return value received through UART1
}
//-----------------------------------------------------------------------------
void DisableAllDriver()
{
//	Packer.Cpu1.HeaterPWM = TempPWM_OFF ;
	u14 = u14PC_CarbonStop ;
//	IsCarbonStart =0;

	u14 = u14PC_PowderCoverStop ;
	u14 = u14PC_DrugCoverStop ;

	u14 = u14PC_PresureStop ;
	u14 = u14PC_PowderStop ;
	u14 = u14PC_DrugStop ;

//	u15 = u15PC_VacuumHighSpeedOFF ;
//	u15 = u15PC_VacuumLowSpeedOFF ;
//	u15 = u15PC_HeaterControlL	;
//	u15 = u15PC_SharkerOFF ;
	u15 = u15PC_PaperResetH ;
	u15 = u15PC_PaperDisable;
}