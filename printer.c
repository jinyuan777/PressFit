#include <C8051F340.h>
#include <communication2.h>
#include <CPU1X.h>
#include <intrins.h>
#include <A_FORM.c>
#include <format.c>
#include <format1.c>
//============================================
#include "USB_API.h"
#include "compiler_defs.h"
#define INTERRUPT_USBXpress 17
#define U32 unsigned int 
xdata TPacker Packer _at_ 0x0000 ;
xdata TUSB	USB;
//............................

SEGMENT_VARIABLE(WriteStageLength, U16, SEG_DATA); //  Current write transfer stage length
SEGMENT_VARIABLE(BytesRead, U16, SEG_DATA);  // Number of Bytes Read
SEGMENT_VARIABLE(M_State, U8, SEG_DATA);     // Current Machine State
SEGMENT_VARIABLE(BytesWrote, U16, SEG_DATA); // Number of Bytes Written
SEGMENT_VARIABLE(ReadIndex, U8*, SEG_DATA);
SEGMENT_VARIABLE(BytesToWrite, U16, SEG_DATA);
idata unsigned  char Buffer[56];
//-----------------------------------------------------------------------------
// UART1 Global Variables
//-----------------------------------------------------------------------------
xdata unsigned int errorTime;

bit IsUSBRxReady;
bit IsDrugReady;
unsigned char RxTimeOutCount=0,CNumber ; // UART 接收超時時間設定
idata unsigned char PNumber;
bit IsCarbonEntry=0;
bit IsCancelPack =0;	
bit IsPrintDate =0;
bit IsPrintNumber =1;

bit IsCut=0;
bit IsFR=0;
bit IsAgain =0;
bit IsShowCarbonEntry=0;
bit CarbonBTW = 0;
unsigned char *Pointer_Packer = &Packer.UART_Mark;

bit	IsRxTimeOut =0;	
bit CounterTag = 0,Dir = 0,CarbonTag = 0;
//bit s = 0;
unsigned char Counter,TempPWM = 0; 
unsigned int DrugStartPosition   = 280 ,CarbonError = 0;
unsigned char IMGSer = 0;
//TSize	PGMode;
//-----------------------------------------------------------------------------
// 主程式
//-----------------------------------------------------------------------------

void ini_u14(){
	_Sel = 1;
	P2=0xEF;
	Delay(20);

	P4=0x90;
	_A1=1;_A0=1;
	_RD=1;
	_WR=0;
	Delay(10);
	_WR=1;
	Delay(20);
	_Sel = 0;
}

void ini_u15(){
	_Sel = 1;
	P2=0xFF;
	_RESET = 1;
	_RESET = 0;
	P4=0x92;
	_A1=1;_A0=1;
	_RD=1;
	_WR=0;
	Delay(10);
	_WR=1;
	Delay(20);
	_Sel = 0;
}

void Powder(unsigned char type,unsigned char pwm)
{
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
	Delay(50);
	PCA0CPH0=pwm;
}
void _u14pb(unsigned char MOD){
	_Sel = 1;
	P2=0xEF;
	P4=MOD;
	_RESET = 0;
	_A1=0;_A0=1;
	_RD=1;
	_WR=0;
	_WR=1;
	_Sel = 0;
}
void _u14pc(unsigned char MOD){
	_Sel = 1;
	P2=0xEF;
	P4=MOD;
	_RESET = 0;
	_A1=1;_A0=0;
	_RD=1;
	_WR=0;
	_WR=1;	
	_Sel = 0;
}
void _u15pc(unsigned char MOD){
	_Sel = 1;
	P2=0xFF;
	P4 = MOD;
	_RESET = 0;
	_RD=1;
	_A1=1;_A0=0;
	_WR=0;
	_WR=1;	
	_Sel = 0;		
}

void _u15Motor(unsigned char MOD){
	_Sel = 1;
	P2=0xFF;
	P4 = MOD;
	_RESET = 0;
	_RD=1;
	_A1=1;_A0=0;
	_WR=0;
	_WR=1;	
	_Sel = 0;		
}
void Cuter(void){
	Delay(6000);	
	_Sel = 1;
	P2=0xEF;
	P4=0x7F;
	_RESET = 0;
	_A1=0;_A0=1;
	_RD=1;
	_WR=0;
	_WR=1;
	_Sel = 0;
	Delay(15000);		
	_Sel = 1;
	P2=0xEF;
	P4=0xFF;
	_RESET = 0;
	_A1=0;_A0=1;
	_RD=1;
	_WR=0;
	_WR=1;
	_Sel = 0;
	Delay(8000);		
}
//-----------------------------------------------------------------------------

void USBPrint(unsigned int Line)
{
	unsigned char i,d;

	#define Bottom  0
	unsigned char tmpEIE1;
	unsigned char tmpEIE2;
	tmpEIE1 = EIE1;
	tmpEIE2 = EIE2;

	EA =0;
	EIE1 = 0;
	EIE2 = 0;

	_u15pc(u15pc |= 0x10);	//	u15 = u15PC_PaperCKH;

	if (Line < Packer.PgMode[USB.PgMode].Left || Line >= Packer.PgMode[USB.PgMode].Right)
	{
		for (i=1;i<=55;i++)
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}
	}
	else 
	{
		for (i=1;i<=55;i++)
		{
			while(!TXBMT);
			SPIF=0;
			if (i<=Bottom || i>=Bottom +55 ) 
				SPI0DAT = 0x00;
			else   					
				SPI0DAT = mem[!IMGSer][Line-Packer.PgMode[USB.PgMode].Left][i-Bottom];
		}
	}

	d=200; while(d--);

	TPHControl();

	EA=1;
	EIE1 = tmpEIE1 ;
	EIE2 = tmpEIE2 ;
	FiveStep();

}
void USBPack(bit IsDrug,bit IsPrint)
{
	unsigned int Line,H;
	Delay(100);

	if (! IsCancelPack)
	{
		// 溫度提高

			Packer.Cpu1.HeaterPWM = TempPWM_Packing ;	
			USB.Command = USBCommand_MemroyWrite;	
			IsUSBRxReady = 0;
			errorTime = Time_USB ;	
			for(Line = 0;Line <Packer.PgMode[USB.PgMode].Length ; Line++)
			{
				if(IsPrint)
				{
				USBPrint(Line);
				if(Line == Packer.PgMode[USB.PgMode].Left)
				{
					Carbon(1);
					u15pc |= 0x08;
				}
				if(Line == Packer.PgMode[USB.PgMode].Right)
				{
					u15pc &= 0xF7;
					Carbon(0);	
				}
				}
				else
				{
					u15 = u15PC_PaperCKH;		
					H = Time_FiveStepDelay;	while(H--);
					u15 = u15PC_PaperCKL;		
					H = Time_FiveStepDelay;	while(H--);
					u15 = u15PC_PaperCKH;		
					H = Time_FiveStepDelay;	while(H--);
					u15 = u15PC_PaperCKL;		
					H = Time_FiveStepDelay;	while(H--);
					u15 = u15PC_PaperCKH;		
					H = Time_FiveStepDelay;	while(H--);
					u15 = u15PC_PaperCKL;		
					H = Time_FiveStepDelay;	while(H--);	
				}
			}
	}	
	Delay(10);
	MainMotorPress();
	while(!IsUSBRxReady&&IsPrint) 
	{
	//	CheckErrorCode(ErrorCode_USBConnection);
		i_Delay(52);
	}
	Packer.Cpu1.ImageNumber ++;
	ClosePrinter();
if(IsDrug)
{
//	USB.Command = USBCommand_DrugsReady;	
//	Packer.Cpu1.State = Cpu1State_Waiting;
	errorTime = Time_USB ;
	while(!IsDrugReady) 
	{
	//	CheckErrorCode(ErrorCode_USBConnection);
		i_Delay(52);
	}
	Packer.Cpu1.State = Cpu1State_Packing;
	Cuter();
	USB.Command = USBCommand_DrugsReady;	
	Packer.Cpu1.State = Cpu1State_Waiting;
	IsDrugReady = 0;
	while(USB.Command == USBCommand_DrugsReady);

	
}

	USB.Command = USBCommand_Idel;
}
//-----------------------------------------------------------------------------
void TPHControl()
{
	unsigned char d;
	_u14pb(0xEF);	//LATCH
	d=10;while(d--);
	_u14pb(0xFF);	

	d = 60;	while(d--);	
	// step 1	

	_u15pc(u15pc &= 0xEF);//	u15 = u15PC_PaperCKL:;
		
	_u14pb(0xFC);//STROBE1,2
	
	Delay(10);

	_u14pb(0xFF);	

	d = 50;	while(d--);	
	// step 2
	_u15pc(u15pc |= 0x10);	//	u15 = u15PC_PaperCKH;

	_u14pb(0xF3);//STROBE3,4	

	Delay(10);
  
	_u14pb(0xFF);
}
//-----------------------------------------------------------------------------
void FiveStep()
{
	unsigned int d;
	u15 = u15PC_PaperCKL;		
	d = Time_FiveStepDelay;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = Time_FiveStepDelay;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = Time_FiveStepDelay;	while(d--);

}

void _u15PAR(){
	_Sel = 1;
	P2=0xFF;
	P4|=0x7F;
	_WR=1;_RD=0;
	_A1=0;_A0=0;
	u15_PA = P4;
	_Sel = 0;
}
void _u15PBR(){
	_Sel = 1;
	P2=0xFF;
	P4|=0xFC;
	_WR=1;_RD=0;
	_A1=0;_A0=1;
	u15_PB = P4;
	_Sel = 0;
}
void _u14PAR(){
	_Sel = 1;
	P2=0xEF;
	P4|=0x0F;
	_WR=1;_RD=0;
	_A1=0;_A0=0;
	u14_PA = P4;
	_Sel = 0;
}

//-----------------------------------------------------------------------------
void Port_Ini(){

    P0MDOUT   = 0x00;
	P1MDOUT   = 0x10;
    P2MDOUT   = 0xFF;
    P3MDOUT   = 0xFF;
    P0SKIP    = 0x01;
	P1SKIP	  = 0xC0;
	P1MDIN    = 0xFF;
    XBR0      = 0x06;
    XBR1      = 0x43;
  //  XBR2      = 0x01;
	P0MDIN   &= ~0x10;

}


void Interrupts_Init()
{
    EIE2      = 0x02;
    IE        = 0x80;
}

void Timer2_ISR (void) interrupt INTERRUPT_TIMER2 
{
	TF2H=0;
	if (RxTimeOutCount ++ >=3)
	{
		TR2 =0;
		TMR2H = 0x00;
		TMR2L = 0x00;
		RxTimeOutCount=0;
		Pointer_Packer =&Packer.Cpu1.Command_UART;
		IsRxCommand =1;
	}
}
void SYSCLK_Init (void)
{
   OSCICN |= 0x03;                     // Configure internal oscillator for
                                       // its maximum frequency
   RSTSRC  = 0x04;                     // Enable missing clock detector
}
void Timer2_Init(){
	TMR2CN = 0x00;
	TMR2L  = 0x00;
	TMR2H  = 0x00;
    ET2 = 1;
}
void EMIF_Init()
{
	EMI0TC = 0xFF;
	EMI0CF = 0x17;
	EMI0CN = 0x00;
}
//-----------------------------------------------------------------------------
void MemoryClear()
{
	unsigned int j,x;

	for (j=0;j<520;j++)
	{
		for(x = 0;x<55;x++)
		{
			mem[1][j][x] = 0x00;
		
		}
	}

}
void MainMotorRun()
{

	if(U14_5){
		while(U14_5)
		{
			PaperMotorForward();			
		}
		IsFR =1;
	}
	else if(U14_6){
		while(U14_6)
		{
			PaperMotorReverse();		
		}
		IsFR =1;
	}
	else if(U14_7){
		PaperMotorLocate();				
		IsFR =1;
	}

}
/*
void DrugLeaveRSW()
{
	bit SWState=0;
	u14 = u14PC_DrugLeft;
	Delay(10);
	PCA0CPH2 = 0x40;
	errorTime = Time_DrugLeaveRSW ;
	while(DrugRSW==0) 
	{
		SWState =1;
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_Drug);
	} 

	if (SWState) Delay(400);
	u14 = u14PC_DrugStop;
	Delay(1600);
}*/
void main()
{   

	unsigned int i=0x0000,time = 1100,d;
	//==============================
    PCA0MD &= ~0x40; 
    USB_Clock_Start();            // Init USB clock *before* calling USB_Init
    USB_Init(USB_VID,USB_PID,USB_MfrStr,USB_ProductStr,USB_SerialStr,USB_MaxPower,USB_PwAttributes,USB_bcdDevice);	
    USB_Int_Enable();             // Enable USB_API Interrupts

	//==============================

	EA = 0;
//	UART1_Init();
	IT01CF = 0x08;
	SPI_Init();
	Port_Ini();
    OSCILLATOR_Init ();                 // Initialize oscillator
	SMB0CF = 0x88;
	SMB0CN = 0x00;
	FLSCL = 0x10;
    PCA0_Init ();                       // Initialize PCA0
	ini_u15();

	ini_u14();
	EMIF_Init();
	Timer0_Init();
	Timer2_Init();
	ADC0_Init();
//	SYSCLK_Init();
	
//	u14pc=0x00;u15pc=0x40;
	u14pb=0xFF;
	_u14pb(u14pb);

	IsFR=1;
	Packer.Cpu1.Command_USB = Cpu1Command_Idel;	
	Packer.Cpu1.Command_UART = Cpu1Command_Idel;	
	Packer.Cpu1.NowNumber =0;
	Packer.Cpu1.PowderLocation =0;
	Packer.Cpu1.Thermometer =50 ;	
	Packer.Cpu1.ImageNumber =0;
	Packer.ErrorCode =0;
	Packer.Cpu1.State = Cpu1State_Heating;
	Packer.Cpu1.PowderStartPosition.W =18;
	Packer.Cpu1.DrugStartPosition.W   =300;
 	Packer.Cpu1.InitTempTime.W =3000;		
	Packer.Cpu1.ShakeArray[0].W = 350;
	Packer.Cpu1.ShakeArray[1].W = 382;		
	Packer.Cpu1.ShakeArray[2].W = 430;		
	Packer.Cpu1.ShakeArray[3].W = 442;		
	Packer.Cpu1.ShakeArray[4].W = 490;		
	Packer.Cpu1.ShakeArray[5].W = 502;		
	Packer.Cpu1.VacuumType = VacuumType_Stop ;
	Packer.Cpu1.ForwardNumber.W =160;			
	Packer.Cpu1.ReverseNumber.W =160;			
	Packer.Cpu1.PowderCoverOpenNumber =2;	
	Packer.Cpu1.DrugCoverOpenNumber   =2;	
	Packer.Cpu1.HeaterPWM = TempPWM_MaxPower;
//	Packer.Cpu1.CarbonPWM =0x10;
	Packer.Cpu1.Message = Message_Heating;
	Packer.ErrorCode = ErrorCode_NoError;
	Packer.UART_Mark = UART_Mark;
	/*
	Packer.PgMode[0].Left 	= 60;
	Packer.PgMode[0].Right 	= 452;
	Packer.PgMode[0].Length = 512;
	Packer.PgMode[0].SpacePg = 4;*/
	Packer.PgMode[0].Left 	= 20;
	Packer.PgMode[0].Right 	= 540;
	Packer.PgMode[0].Length = 576;
	Packer.PgMode[0].SpacePg = 3;

	Packer.PgMode[1].Left 	= 60;
	Packer.PgMode[1].Right 	= 580;
	Packer.PgMode[1].Length = 640;
	Packer.PgMode[1].SpacePg = 3;
	Packer.PgMode[2].Left 	= 180;
	Packer.PgMode[2].Right 	= 700;
	Packer.PgMode[2].Length = 853;
	Packer.PgMode[2].SpacePg = 2;
	USB.Command = USBCommand_Idel;

	EA = 1;


	DisableAllDriver();
	MemoryClear();

	Packer.Cpu1.HeaterPWM = TempPWM_Standby;

	#if !IsPowderOnly

		DrugToRight();		

		DrugCoverOpen();	
	#endif

	while(1){

		byte Command ;
		Cpu1CommandIdel();
		Packer.Cpu1.State = Cpu1State_Idel;
/*
		while(Cpu1Command() == Cpu1Command_Idel && !IsPacking)
		{
			AccessCpu0(Cpu0Command_ShowInfo);

			#if IsMainMotorButton
				//MainMotorRun();
			#endif

			if (RHall != 0 )
			{
				Delay(3000);					
				//PowderLocate();
			}
			Delay(30000);
		}
*/
		//SpacePacking(1);
		Command = Cpu1Command() ;
		Packer.Cpu1.State = Cpu1State_Busy;	
//		AccessCpu0(Cpu0Command_ShowInfo);

		if(Command == Cpu1Command_PackStart)
		{
			Packing();	
			Delay(200);
			Packer.Cpu1.Command_UART = Cpu1Command_Idel;
		}
		else if (Command == Cpu1Command_PaperMotorLocate)
		{
			time = 1000;
			u15 = u15PC_PaperCW ;
			Delay(10);
			u15 = u15PC_PaperEnable;
			Delay(10);
			for (i=0;i<640;i++)
			{
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
			}
			u15 = u15PC_PaperDisable;		
			//Cuter();
			MainMotorPress();
		}
		else if(Command == Cpu1Command_MototPressFit)
		{
			u15 = u15PC_PaperPressON;
			Delay(10);
			errorTime = Time_PresurePS ;
			do 
			{
				i_Delay(Time_PSDelay);
				CheckErrorCode(ErrorCode_PresureDown);	
			} while(PowderCounterPS==1);
			
			Delay(500);
			do 
			{
				i_Delay(Time_PSDelay);
				CheckErrorCode(ErrorCode_PresureDown);	
			} while(PowderCounterPS==0);
			u15 = u15PC_PaperPressOFF;
			Delay(100);
		}
		else if(Command == Cpu1Command_USBPackStart)
		{
			USBPacking();
			//_u14pb(0x7F);//STROBE1,2
			Delay(100);
			//_u14pb(0xFF);//STROBE1,2
		}
		Delay(1000);

	}
}
void MainMotorPress()
{
	u15 = u15PC_PaperPressON;
	Delay(10);
	errorTime = Time_PresurePS ;
	do 
	{
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PowderCounterPS==1);
	
	Delay(500);
	do 
	{
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PowderCounterPS==0);
	u15 = u15PC_PaperPressOFF;
	Delay(100);
}
//-----------------------------------------------------------------------------
unsigned char Read_S=0x00, Mark = 0xFF;
void USBPacking() 
{

	Packer.Cpu1.State = Cpu1State_Packing;
	//AccessCpu0(Cpu0Command_ShowLeftDownGroupBox);

	//CheckPauseState(1);

	if (! IsCancelPack)
	{
	

		PackForUSB();
		IsPacking =0; 

	}


	Delay(2000);
	Packer.Cpu1.State = Cpu1State_Idel;
	Packer.Cpu1.NowNumber =0;
	//AccessCpu0(Cpu0Command_ShowPackFinish);

	Packer.Cpu1.Message = Message_PackFinish;

	Delay(4000);
	MemoryClear();
	//AccessCpu0(Cpu0Command_ShowSystemReady);
	Packer.Cpu1.Message = Message_SystemReady;		

	Packer.Cpu1.HeaterPWM = TempPWM_Standby ;		
	Cpu1CommandIdel();	
}
//-----------------------------------------------------------------------------
void DrugCoverOpen()
{
	unsigned char i;
	i = Packer.Cpu1.DrugCoverOpenNumber ;
	if (i>4) i=2;

	u14 = u14PC_DrugCoverStart ;
	Delay(50);
	while(i--)
	{
		errorTime = Time_DrugCoverPS ;
		do 
		{
			i_Delay(Time_PSDelay);
			CheckErrorCode(ErrorCode_DrugCover);
		} while(DrugCoverPS==1) ;

		errorTime = Time_DrugCoverPS ;
		do 
		{
			i_Delay(Time_PSDelay);
			CheckErrorCode(ErrorCode_DrugCover);
		} while(DrugCoverPS==0) ;
	}
	u14 = u14PC_DrugCoverStop ;
	Delay(800);
}
//-----------------------------------------------------------------------------

void PackForUSB()
{
	unsigned char PackNum = USB.PackNumber + Packer.PgMode[USB.PgMode].SpacePg;

	Packer.Cpu1.NowNumber = 0;
	//-----------------------------------------------
	
	USB.Command = USBCommand_MemroyWrite;	
	IsUSBRxReady = 0;
	errorTime = Time_USB ;
	while(!IsUSBRxReady) 
	{
	//	CheckErrorCode(ErrorCode_USBConnection);
		i_Delay(52);
	}


	//-----------------------------------------------
	u15 = u15PC_PaperCW ;
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
	Carbonini();
	IsPacking =1;

	Packer.Cpu1.ImageNumber =0;
	USB.Command = USBCommand_DrugsReady;	
	Packer.Cpu1.State = Cpu1State_Waiting;	
	IsDrugReady = 0;
	MainMotorPress();
    IsShake = USB.IsShake;
	for (PNumber=1; PNumber <= PackNum; PNumber++)
	{
		if(PNumber == PackNum)
		{
			Packer.Cpu1.NowNumber = PNumber-2 ;
			Packer.Cpu1.ImageNumber =0;

			USBPack(0,0);			
			if (IsCancelPack)
			{
				return ;
			}
		}
		else if (PNumber > (PackNum-Packer.PgMode[USB.PgMode].SpacePg))		
		{
			Packer.Cpu1.NowNumber = PNumber-2 ;
			Packer.Cpu1.ImageNumber =0;

			USBPack(1,0);			
			if (IsCancelPack)
			{
				return ;
			}
		}

		else if (PNumber <=Packer.PgMode[USB.PgMode].SpacePg)						
		{

			Packer.Cpu1.NowNumber = 0;

			USBPack(0,1);			
							
			if (IsCancelPack)
			{
				return ;
			}
		}

		else if (PNumber > Packer.PgMode[USB.PgMode].SpacePg && PNumber <= PackNum-Packer.PgMode[USB.PgMode].SpacePg)
		{

			Packer.Cpu1.NowNumber = PNumber-2 ;	
			USBPack(1,1);			
			if (IsCancelPack)
			{
				return ;
			}
		}

	}
	Cuter();
	USBPack(0,0);
	USBPack(0,0);
	USBPack(0,0);
	Packer.Cpu1.Message = Message_PackSpace;

	u15 = u15PC_PaperDisable;
	Delay(10);

}
void SpacePacking(byte SpaceNumber)
{
	for (PNumber=1; PNumber <= SpaceNumber; PNumber++)	// 空包
	{
		Packer.Cpu1.NowNumber = PNumber ;
		SpacePack();
		if (IsCancelPack)
		{
			return ;
		}
			Delay(500);
			MainMotorPress();
	}	
}

//-----------------------------------------------------------------------------
void CheckPauseState(bit IsPrint)
{
//	unsigned char tag=0;

	Packer.Cpu1.State = Cpu1State_Packing;
//	AccessCpu0(Cpu0Command_ShowPackInfo);

	IsCancelPack =0;
	if (PaperEntrySW )
	{
		Packer.Cpu1.State = Cpu1State_PackPause ;
		AccessCpu0(Cpu0Command_ShowPaperEntry);
		Packer.Cpu1.Message = Message_PaperEntryAlerm ;
//		tag = 1;
	}

	if (IsPackPause)
	{
		Packer.Cpu1.State = Cpu1State_PackPause ;
		AccessCpu0(Cpu0Command_ShowPackPause);
		Packer.Cpu1.Message = Message_PackPause ;
	}

    if (IsCarbonEntry && IsPrint) 
	{
		IsShowCarbonEntry =1;
		Packer.Cpu1.State = Cpu1State_PackPause ;
		AccessCpu0(Cpu0Command_ShowCarbonEntry);
		Packer.Cpu1.Message = Message_CarbonEntryAlerm ;

	}
	
	else 
	{
		if (!IsShowCarbonEntry)
		{
			AccessCpu0(Cpu0Command_ShowCarbonNoEntry);
//			AccessUSB(USBCommand_ShowCarbonNoEntry);
		}
	}

	if (PaperEntrySW ==1 || IsPackPause || IsShowCarbonEntry) 
	{
		Packer.Cpu1.State = Cpu1State_PackPause ;
		AccessCpu0(Cpu0Command_ShowInfo) ;
		u15 = u15PC_PaperDisable;
		Delay(10);
		u15 = u15PC_SharkerOFF ;
		ClosePrinter();
		if (IsPrint)
		{
			Carbon(0);
			Delay(10);
			//PresureDown();
		}
		
		while(1)
		{			
			Packer.Cpu1.State = Cpu1State_PackPause ;
			AccessCpu0(Cpu0Command_ShowInfo) ;	

			if (Cpu1Command() == Cpu1Command_PackContinue && PaperEntrySW ==0) {
				Cpu1CommandIdel() ;	
				Packer.Cpu1.Message = Message_SystemReady ;
				PaperMotorLocate();	
				ShowPackType();
				CNumber += 2;
				PNumber -= 1;
				u15 = u15PC_PaperEnable;
				if (IsPrint)
				{
					//PresureUp();
					Carbon(1);
				}
				Carbonini();
				IsShowCarbonEntry = 0;
				IsCancelPack =0;
				Packer.Cpu1.State = Cpu1State_Packing ;
				AccessCpu0(Cpu0Command_ShowPackContinue);
				break ;
			}	
			// 停止包裝
			else if (Cpu1Command() == Cpu1Command_PackStop) 
			{
				Cpu1CommandIdel() ;
				Packer.Cpu1.State = Cpu1State_Idel ;
				AccessCpu0(Cpu0Command_ShowPackStop);
				Carbonini();
				IsCancelPack =1;
				Packer.Cpu1.Message = Message_SystemReady ;
				IsPacking =0;
				break ;
			}
			#if IsMainMotorButton
				MainMotorRun();
			#endif
			Delay(30000);
		}

		AccessCpu0(Cpu0Command_ShowPaperReady);
	}
}
//-----------------------------------------------------------------------------
void Packing()
{

	Packer.Cpu1.State = Cpu1State_Packing;
	AccessCpu0(Cpu0Command_ShowLeftDownGroupBox);

	AccessCpu0(Cpu0Command_TxPackInfo);

	CheckPauseState(0);
	if (! IsCancelPack)
	{
		if (PaperMotorPS != 1)
		{
			PaperMotorLocate();		
		}

		Packer.Cpu1.HeaterPWM = TempPWM_Packing ;
		
	
//		if (IsAutoOpen)
		{
			if (Packer.Cpu0.PackType == PackType_Powder || 	Packer.Cpu0.PackType == PackType_Mix) 
			{

				PowderCoverOpen();		
				
				//Packer.Cpu1.VacuumType = VacuumType_LowSpeed;
			}
			if (Packer.Cpu0.PackType == PackType_Drug || Packer.Cpu0.PackType == PackType_Mix) 
			{
				DrugCoverOpen();		
			}
		}
		IsPacking =1;
		if (IsPrintPack ) 	
		{
			PackWithPrint();		
		}
		else				
		{
			PackWithoutPrint();
		}
		IsPacking =0;	
		#if !IsPowderOnly
			DrugToRight();	
		#endif				
		IsPacking =0;
		if ( Packer.Cpu0.PackType == PackType_Powder || 
		     Packer.Cpu0.PackType == PackType_Mix ) 
		{
			if (IsCancelPack)
			{ 
				Packer.Cpu1.VacuumType = VacuumType_Stop ;
				PowderNumber();
			}
			else
			{
				AutoClear(); 
			}
		}

	}

	Delay(1000);
	Packer.Cpu1.State = Cpu1State_Idel;
	Packer.Cpu1.NowNumber =0;
	AccessCpu0(Cpu0Command_ShowPackFinish);
//	AccessUSB(USBCommand_Idel);
	Packer.Cpu1.Message = Message_PackFinish;	
	Delay(8000);
	MemoryClear();
	AccessCpu0(Cpu0Command_ShowSystemReady);
	Packer.Cpu1.Message = Message_SystemReady;			
	Packer.Cpu1.HeaterPWM = TempPWM_Standby ;		
	Cpu1CommandIdel();
}
//-----------------------------------------------------------------------------
void PackWithoutPrint()
{
	unsigned char PNumber;
	u15 = u15PC_PaperResetH;
	Delay(10);
	u15 = u15PC_PaperCW ;
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
	IsPacking =1;

	// 包裝藥粉.錠劑.混合
	ShowPackType();
	for (PNumber=1 ; PNumber <= Packer.Cpu1.PowderLocation ; PNumber++) 	
	{
		Packer.Cpu1.NowNumber =PNumber;
		OnePack(Packer.Cpu0.PackType , PrintFalse,IsCut);
		if(PNumber==1)
			IsCut=1;
		else
			IsCut=0;
		if (IsCancelPack)
		{
			// 取消包裝
			return ;
		}
	}

	AccessCpu0(Cpu0Command_ShowPackSpace);

	#if IsCuterOk
		if(IsAutoCut)
		{
			SpacePacking(1);
			Delay(5000);
			_One(PackType_Space ,PrintFalse);
			Delay(5000);
			// 主馬達定位
			PaperMotorLocate();
		}	
		else
			SpacePacking(Packer.Cpu0.SpaceNumber);
	#else 
		SpacePacking(Packer.Cpu0.SpaceNumber);		
	#endif	
//	PaperMotorLocate();

	u15 = u15PC_PaperDisable;
	Delay(10);
}
//========================================================
void _One(unsigned char Type,bit IsPrint)
{
	unsigned int n;

	CheckPauseState(IsPrint);
	u15 = u15PC_PaperEnable;
	Delay(10);
	if (! IsCancelPack)
	{
		for (n=0;n< CutterPosition ;n++)
		{
			OneStep(Time_PaperMotorDelay);	
		}

	Delay(15000);
			
	Cuter();
			
	u15 = u15PC_SharkerOFF ;
	ClosePrinter();
	}
}
void OnePack(unsigned char Type,bit IsPrint,bit IsCuts)
{
	unsigned int n;

	CheckPauseState(IsPrint);
	if (! IsCancelPack)
	{
	
		PCA0CPH0 = 0x60;
		for (n=0;n< 640 ;n++)
		{
//			TR1 = 0;
			if (IsPrint) 
			{
				Print(n,PNumber-1);
				if(n==CutterPosition&&IsCuts==1){
				#if IsCuterOk
					if(IsAutoCut)
					{
						
							Carbon(0);
							Delay(15000);
							EA = 0;Cuter();EA = 1;
							Delay(10000);
							Carbon(1);			
					
					}
				#endif
				}
			}

			else 	 
			{
				OneStep(Time_PaperMotorDelay-40);
				if(n==CutterPosition&&IsCuts==1){
				#if IsCuterOk
					if(IsAutoCut)
					{						
						
							Delay(15000);
							EA = 0;Cuter();EA = 1;
							Delay(10000);		
												
					}
				#endif	
				}
			}
			if (Type == PackType_Drug )		
			{
				// 錠劑
				if (n == DrugStartPosition) 
					IsDrugMove =1;
			}
			
			else if (Type == PackType_Mix) 	
			{
				// 混合 
				if (n == DrugStartPosition)   
					IsDrugMove   =1;
				Shake(n) ;
			}
		}

	u15pc &= 0xF7;

	ClosePrinter();
	}
}
//-----------------------------------------------------------------------------
void Shake(unsigned int n)
{
	if (IsShake ==0) return;
	if ( n == Packer.Cpu1.ShakeArray[0].W || 
		 n == Packer.Cpu1.ShakeArray[2].W || 
		 n == Packer.Cpu1.ShakeArray[4].W )
	{
		//u15 = u15PC_SharkerON ;
		u15pc |= 0x08;
	}
	else if ( n == Packer.Cpu1.ShakeArray[1].W || 
		  	  n == Packer.Cpu1.ShakeArray[3].W || 
			  n == Packer.Cpu1.ShakeArray[5].W )
	{
		//u15 = u15PC_SharkerOFF ;
		u15pc &= 0xF7;
	}
	//Delay(10);
}
//-----------------------------------------------------------------------------
void Carbonini()
{
	CarbonTag = CarbonHall;
	CarbonError = 0;
	IsCarbonEntry = 0;

}
//-----------------------------------------------------------------------------
void PackWithPrint()
{
	unsigned char PackTotal = Packer.Cpu1.PowderLocation +2;

	HeadWithPrint();

	//PresureUp();

	Carbon(1);
	u15 = u15PC_PaperCW ;
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
	Carbonini();
	IsPacking =1;

	ShowPackType();
	for (PNumber=1;PNumber<= PackTotal;PNumber++)
	{
		if(PNumber == 4)
			IsCut = 1;
		else
			IsCut = 0;
		if (PNumber == PackTotal-1)	
		{
			Carbon(0);
			Delay(10);
			
		//	PresureDown();	

			Packer.Cpu1.NowNumber =PNumber-2 ;		

			OnePack(Packer.Cpu0.PackType , PrintFalse,IsCut);
			if (IsCancelPack)
			{
				return ;
			}
		}
		else if (PNumber >= PackTotal)		
		{
			Packer.Cpu1.NowNumber =PNumber-2 ;		
			OnePack(Packer.Cpu0.PackType , PrintFalse,IsCut);
			if (IsCancelPack)
			{
				return ;
			}
		}
		else if (PNumber <=2 )					
		{
			Packer.Cpu1.NowNumber =0;

			OnePack(PackType_Space , PrintTrue,IsCut);
			if (IsCancelPack)
			{
				return;
			}
		}
		else if (PNumber > 2 && PNumber < PackTotal-1)
		{
			Packer.Cpu1.NowNumber =PNumber-2 ;		
			
			OnePack(Packer.Cpu0.PackType , PrintTrue,IsCut);
			if (IsCancelPack)
			{
				return;
			}
		}


	}
	#if IsCuterOk
		if(IsAutoCut)
		{
			SpacePacking(1);
			Delay(5000);
			_One(PackType_Space ,PrintFalse);
			Delay(5000);
			// 主馬達定位
			PaperMotorLocate();
		}	
		else
			SpacePacking(Packer.Cpu0.SpaceNumber);
	#else 
		SpacePacking(Packer.Cpu0.SpaceNumber);		
	#endif	
//	PaperMotorLocate();	// 主馬達定位

	u15 = u15PC_PaperDisable;
	Delay(10);
}
//-----------------------------------------------------------------------------
void Print(unsigned int Line,signed int Number)
{
	unsigned char i;
	unsigned char tmpEIE1;
	unsigned char tmpEIE2;

	unsigned int Shift = 0 ;
	
	u15 = u15PC_PaperCKH;
	tmpEIE1 = EIE1;
	tmpEIE2 = EIE2;
//	EA =0;
//	EIE1 = 0;
//	EIE2 = 0;


	if (Line < Packer.PgMode[0].Left || Line >= Packer.PgMode[0].Right)
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
		unsigned char n,k;
		for (i=0 ; i < 21 ; i++)
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}

		if (IsPrintDate)
		{
			for (i=0 ; i < 4 ; i++)
			{
				while(!TXBMT);
				SPIF=0;
				if (Line >=Shift && Line < Shift+32 )
				{
					unsigned char n = 2;
					SPI0DAT = cA11[n][Line - Shift][i] ;
				}
				else if (Line >=Shift +32 && Line < Shift+64 )
				{
					unsigned char n = 0;
					SPI0DAT = cA11[n][Line - Shift -32][i] ;
				}
				else if (Line >=Shift +64 && Line < Shift+96 )
				{
					unsigned char n = 1;
					SPI0DAT = cA11[n][Line - Shift -64][i] ;
				}
				else if (Line >=Shift +96 && Line < Shift+128 )
				{
					unsigned char n = 1;
					SPI0DAT = cA11[n][Line - Shift -96][i] ;
				}
				else if (Line >=Shift +128 && Line < Shift+160 )
				{
					SPI0DAT = cA11[10][Line - Shift -128][i] ;
				}
				
				else if (Line >=Shift +160 && Line < Shift+192 )
				{
					unsigned char n = 0;
					SPI0DAT = cA11[n][Line - Shift -160][i] ;
				}
				else if (Line >=Shift +192 && Line < Shift+224 )
				{
					unsigned char n = 3;
					SPI0DAT = cA11[n][Line - Shift -192][i] ;
				}
				else if (Line >=Shift +224 && Line < Shift+256)
				{
					SPI0DAT = cA11[10][Line - Shift -224][i] ;
				}

				else if (Line >=Shift +256 && Line < Shift+288 )
				{
					unsigned char n = 0;
					SPI0DAT = cA11[n][Line - Shift -256][i] ;
				}
				else if (Line >=Shift +288 && Line < Shift+320 )
				{
					unsigned char n = 9;
					SPI0DAT = cA11[n][Line - Shift -288][i] ;
				}
				else SPI0DAT = 0x00;
			}
		}

		if (IsPrintNumber)
		{
			for (i=0 ; i < 4 ; i++)	
			{
				while(!TXBMT);
				SPIF=0;
				if (Line >=Shift && Line < Shift+32 )
				{
					unsigned char n = (Number +1) /10;
					SPI0DAT = cA11[n][Line - Shift][i] ;
				}
				else if (Line >=Shift +32 && Line < Shift+64 )
				{
					unsigned char n = (Number +1) %10;
					SPI0DAT = cA11[n][Line - Shift -32][i] ;
				}
				else if (Line >=Shift +64 && Line < Shift+96 )
				{
					SPI0DAT = cA11[10][Line - Shift -64][i] ;
				}
				else if (Line >=Shift +96 && Line < Shift+128 )
				{
					unsigned char n = Packer.Cpu0.TotalNumber /10;
					SPI0DAT = cA11[n][Line - Shift -96][i] ;
				}
				else if (Line >=Shift +128 && Line < Shift+160 )
				{
					unsigned char n = Packer.Cpu0.TotalNumber %10;
					SPI0DAT = cA11[n][Line - Shift -128][i] ;
				}
				else SPI0DAT = 0x00;
			}
		}

		for (i=0 ; i < 9 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0;
		}

		k =0;
		for (i=0; i<=3; i++)		// 4
		{
			if ( Packer.Cpu0.PrintItem[i] != 0 ) k++;
		}

		n =	(Number % k) *8  ;

		for (i=0 ; i < 8 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = mem[Line][n+i];
		}

		for (i=0 ; i < 4 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}

		for (i=0 ; i < 4 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			
			if (IsPatientID)	
			{
				SPI0DAT = mem[Line][i+36];
			}
			else 			SPI0DAT = 0x00;
		}


		for (i=0 ; i < 4 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			if (IsDateTime)	
			{
				SPI0DAT = mem[Line][i+32];
			}
			else 			SPI0DAT = 0x00;
		}

		for(i=0;i<7;i++){
			 while(!TXBMT);
			SPIF=0;		
			SPI0DAT = 0x00;
		}

		for (i=0 ; i < 7; i++)
		{
			while(!TXBMT);
			SPIF=0;
			if (IsName && Line>138&&Line<330) SPI0DAT = CC[Line-138][i];
			else 		
				SPI0DAT = 0x00;
		}

		for (i=0 ; i < 9 ; i++)	
			{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}
	}

	TPHControl();


//	EA=1;
//	EIE1 = tmpEIE1 ;
//	EIE2 = tmpEIE2 ;
	FiveStep();
}

void SpacePack()
{
	PaperMotorLocate();	
}
/*
void EightStep()
{
	unsigned int d;
	u15 = u15PC_PaperCKH;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKH;		
	d = 1200;	while(d--);
	u15 = u15PC_PaperCKL;		
	d = 1200;	while(d--);
}*/
//-----------------------------------------------------------------------------
void ClosePrinter()
{
	u14pb = 0xFF;
}
//===============================================

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
unsigned int IMGTAG = 0x0000;
void  Receive_Setup (void)
{
   char i = 0;
   BytesRead   =  Block_Read(&Buffer,  56);      // Read Setup Message


   if (Buffer[0]  == READ_MSG)         // Check See if Read File Setup
   {
   	  Packer.Cpu1.Command_USB   = Buffer[1];
	  USB.PackType		= Buffer[2]; 
	  USB.SpaceNumber	= Buffer[3];
	  USB.PackNumber	= Buffer[4];
	  USB.StartNumber	= Buffer[5];
	  USB.TotalNumber	= Buffer[6];
	  USB.PgMode		= Buffer[7];
	  USB.IsShake		= Buffer[8];
	  USB.Year			= Buffer[9];
	  USB.Month			= Buffer[10];
	  USB.Day			= Buffer[11];
	  USB.IsAutoCut	= Buffer[12];
	  USB.IsPrintNumber	= Buffer[13];
	  IsDrugReady		= Buffer[14];
      Buffer[0]   =  Mark;            // Send host size of transfer message
   	  Buffer[1]	  =  USB.Command;
      Buffer[2]   =  Packer.Cpu1.State;//Read_S; //LengthFile[1];
      Buffer[3]   =  Packer.Cpu1.NowNumber;//Signs;  //LengthFile[0];
	  Buffer[4]	  =  Packer.Cpu1.PowderLocation;//Packer.Cpu1.PowderLocation;
   	  Buffer[5]	  =  Packer.Cpu1.Thermometer;
   	  Buffer[6]	  =  Packer.Cpu1.ImageNumber;
	  Buffer[7]   =  Packer.Cpu1.Message;
	  Buffer[8]	  =  Packer.ErrorCode;
	  Buffer[9]	  =  Read_S;
	  Buffer[10]  =  0xFF;	  
      BytesWrote  =  Block_Write((unsigned char*)&Buffer,   15);
	  M_State =  ST_IDLE_DEV; 
	  Read_S+=1;
	  Mark = 0xFF;

   }
   else if(Buffer[0] == WRITE_DAT)
   {
		for(i=1;i<56;i++)
		{
			mem[IMGSer][IMGTAG][i-1] = Buffer[i];
		}
		IMGTAG ++;
		if(IMGTAG >= 520)
		{
			USB.Command = USBCommand_Idel;
			IsUSBRxReady = 1;
			IMGTAG = 0;
			if(IMGSer == 0)
				IMGSer = 1;
			else
				IMGSer = 0;
		}
		else
			USB.Command = USBCommand_MemroyWrite;
		Buffer[1] = USB.Command;
		BytesWrote  =  Block_Write((unsigned char*)&Buffer,   15);
	  	M_State =  ST_IDLE_DEV;	 		
   }
   else if(Buffer[0] == CommandACK)
   {
   		USB.Command = USBCommand_Idel;
		Buffer[1] = USB.Command;
		BytesWrote  =  Block_Write((unsigned char*)&Buffer,   15);
	  	M_State =  ST_IDLE_DEV;	 
   }
}

//===============================================
//-----------------------------------------------------------------------------
void PowderToLeft()
{
	Packer.Cpu1.State = Cpu1State_PowderLocating;
	Powder(MotorLeft,PowderPWM);	

	errorTime = Time_PowderLSW ;

	while (PowderLSW)
	{
//		Delay(1000);
//		Delay(1000);
		i_Delay(Time_PSDelay);
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_Powder);
	}
	Powder(MotorStop,0xFF);
	Delay(1000);
	Counter = Ini_Location;
	CounterTag = PowderCounterPS;
}
//-----------------------------------------------------------------------------
void PowderToRight()
{
	Packer.Cpu1.State = Cpu1State_PowderLocating;
	Powder(MotorRight,PowderPWM);
	
	errorTime = Time_PowderRSW ;

	while (PowderRSW)
	{
//		Delay(1000);
//		Delay(1000);
		i_Delay(Time_PSDelay);
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_Powder);
	}
	Powder(MotorStop,0xFF);
	Delay(1000);
}
void DrugToRight()
{
	PCA0CPH2 = 0x40;
	u14 = u14PC_DrugRight;
	Delay(20);

	errorTime = Time_DrugRSW ;

	while(DrugRSW==1) 
	{
		i_Delay(Time_PSDelay);
		CheckErrorCode(ErrorCode_Drug);
	} 

	u14 = u14PC_DrugStop;
	Delay(800);
}
//-----------------------------------------------------------------------------
void CloseLeave()
{
	Powder(MotorLeft,PowderPWM);
	while(PowderCounterPS ==1 ) 
	{
		if (PowderLSW ==0) 
		{
		Powder(MotorStop,0xFF);
		Delay(1000);
		return;
		}

	}
	while(PowderCounterPS ==0 ) 
	{
		if (PowderLSW ==0) 
		{
		Powder(MotorStop,0xFF);
		Delay(1000);
		return;
		}

	}

	Powder(MotorStop,0xFF);
	Powder(MotorLeft,PowderPWM);
	while(PowderCounterPS ==1) 
	{
		if (PowderLSW ==0) 
		{
		Powder(MotorStop,0xFF);
		Delay(1000);
		return;
		}

	}
	while(PowderCounterPS ==0) 
	{
		if (PowderLSW ==0) 
		{
		Powder(MotorStop,0xFF);
		Delay(1000);
		return;
		}
	}

	Powder(MotorStop,0xFF);
	Delay(1000);

	Powder(MotorRight,PowderPWM);
	while(PowderCounterPS ==1) ;
	while(PowderCounterPS ==0) ;
	Powder(MotorStop,0xFF);
	Powder(MotorRight,PowderPWM);
	while(PowderCounterPS ==1) ;
	while(PowderCounterPS ==0) ;
	Powder(MotorStop,0xFF);

	IsFR =0;
	Delay(1000);
}
//-----------------------------------------------------------------------------
void PowderNumber()
{

	unsigned char s=0;
	Packer.Cpu1.PowderLocation =0;		
	Packer.Cpu1.State = Cpu1State_PowderLocating;
	AccessCpu0(Cpu0Command_ShowPowderLocating);	
	Packer.Cpu1.Message = Message_PowderLocating;		
	//AccessUSB(USBCommand_Busy);	

	if (PaperMotorPS != 1)
	{
		PaperMotorLocate();		
	}


	PowderToLeft();						


	PowderCounterPS_LastState = PowderCounterPS ;
	PowderLocationCounter = 0;
	PowderLocate();						
}

//-----------------------------------------------------------------------------
void Carbon(bit bSW){
	CarbonBTW = bSW;
	PCA0CPH1  = 0xA0;
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

void PaperMotorForward()
{
	unsigned int i;
	u15 = u15PC_PaperCW ;
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
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
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
	for (i=0 ; i < 80 ; i++)
	{
		OneStep(800);	
	}
	u15 = u15PC_PaperDisable;
}
//-----------------------------------------------------------------------------
void PaperMotorLocate()
{
	unsigned char i;
	u15 = u15PC_PaperCW ;
	Delay(10);
	u15 = u15PC_PaperEnable;
	Delay(10);
	for (i=0;i<Packer.PgMode[USB.PgMode].Length;i++)
		OneStep(Time_PaperMotorDelay);	
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
   PCA0CPM1 = 0x42;
   PCA0CPM2 = 0x42;
   // Configure initial PWM duty cycle = 50%
   PCA0CPH0 = 0x60;
   PCA0CPH1 = 0x40;
   PCA0CPH2 = 0x40;
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
}

//-----------------------------------------------------------------------------
void SPI_Init()
{
    SPI0CFG   = 0x40;
    SPI0CN    = 0x01;
 	SPI0CKR   = 0x07;
}
void Timer0_Init(void)
{
   TH0 = 0x30;//-31.25;           // Init Timer0 High register
   TL0 = TH0;                          // Set the intial Timer0 value
   TH1 = 0x30;
   TL1 = TH1;
   TMOD = 0x02;                        // Timer0 in 8-bit reload mode
   CKCON = 0x22;                       // Timer0 uses a 1:48 prescaler
   ET0 = 1;                              // Timer0 interrupt enabled
   ET1 = 1;
   TCON = 0x50;                        // Timer0 ON
}
void Timer1_ISR (void) interrupt 3
{
	TR1 = 0;
	/*
	if(Packer.Cpu1.VacuumType == VacuumType_HighSpeed || VacuumSW)
		u15pc &= 0xFE;
	else if(Packer.Cpu1.VacuumType == VacuumType_LowSpeed)
		u15pc &= 0xFD;
	else if(Packer.Cpu1.VacuumType == VacuumType_Stop || !VacuumSW)
		u15pc |= 0x03;*/
	TempPWM ++;
	if(CarbonBTW)
	{
		if(CarbonHall != CarbonTag)
		{
			CarbonTag = CarbonHall;
			CarbonError = 0;
		}
		else
		{			
			if(CarbonError ++ > 500)
			{
				IsCarbonEntry = 1;
				CarbonError = 0;
			}
		}
	}
	AD0BUSY = 1;
	TR1 = 1;
}
void Timer0_ISR (void) interrupt 1
{
	TR0=0;
//	P2 &= 0x7F;
	_Sel = 1;
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
			//PCA0CPH0=0x60;
			u14pc &= 0xEF;
			break;
		case u14PC_PowderLeft:
			//PCA0CPH0=0x60;
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
		case u15PC_PaperCW:
			u15pc &= 0xDF;
			break;
		case u15PC_PaperCCW:
			u15pc |= 0x20;
			break;
		case u15PC_PaperEnable:
			u15pc &= 0xBF;
			break;
		case u15PC_PaperDisable:
			u15pc |= 0x40;
			break;
		case u15PC_PaperPressON:
			u15pc &= 0xFD;
			break;
		case u15PC_PaperPressOFF:
			u15pc |= 0x03;
			break;
		case u15PC_SharkerON:
			u15pc |= 0x08;
			break;
		case u15PC_SharkerOFF:
			u15pc &= 0xF7;
			break;

	}
	if(TempPWM ++ > 0XFE)
		TempPWM = 0;
	if(TempPWM < Packer.Cpu1.HeaterPWM)
	{
		u15pc &= 0xFB;
	}
	else 
	{
		u15pc |= 0x04;	
	}
//	_u14pb(u14pb);
	_u14pc(u14pc);
	_u15pc(u15pc);
	_u14PAR();
	_u15PAR();
	/*
	if(PaperLPS == !PaperLSign && PaperLPS == 0)
		PaperLTension ++;	
    if(PaperRPS == !PaperRSign && PaperRPS == 0)
		PaperRTension ++;
	PaperLSign = PaperLPS;	
	PaperRSign = PaperRPS;*/
	_u15PBR();

	_Sel = 0;
	TR0=1;

}

//-----------------------------------------------------------------------------
void PowderLocate()
{
	Packer.Cpu1.State = Cpu1State_PowderLocating;
	AccessCpu0(Cpu0Command_ShowPowderLocating);			
	Packer.Cpu1.Message = Message_PowderLocating;
	while(1)
	{
		if(!PowderLSW)
		{
			Powder(MotorStop,0xFF);
			Delay(1000);
			CursorLocation = CursorRightEntry;
			Dir = 1;
			Powder(MotorRight,PowderPWM);
			//Delay(1000);
			while(!PowderLSW);
		}
		else if(!PowderRSW)
		{
			Powder(MotorStop,0xFF);
			Delay(1000);
			//Counter = 90;
			CursorLocation = CursorLeftEntry;
			
			Powder(MotorLeft,PowderPWM);
			//Delay(1000);
			while(!PowderRSW);
		}
		if ( CursorLocation == CursorLeftEntry)	//1
		{
			Powder(MotorLeft,PowderPWM);
			Dir = 0;
		}
		else if ( CursorLocation == CursorRightEntry)	//1
		{
			Powder(MotorRight,PowderPWM);
			Dir = 1;
		}
		else if ( CursorLocation == CursorLeftHall )	//2
		{
			unsigned long l=0x60000;
			Powder(MotorStop,0xFF);
			Delay(5000);
			Dir = 1;
			Powder(MotorRight,0xA0);		
			while (CursorLocation != CursorRightHall)
			{				
				if (l-- ==0 || !PowderRSW)
				{	
					Powder(MotorStop,0xFF);
					Delay(5000);
					CounterTag = PowderCounterPS;
					break;
				}
			}
			if (CursorLocation == CursorRightHall){
				break;
			}
		}

	}
	Powder(MotorStop,0xFF);
	Delay(6000);
	Packer.Cpu1.PowderLocation = Counter/2;
	CounterTag = PowderCounterPS;
	if(!PowderCounterPS == 0){
		Packer.ErrorCode = ErrorCode_Powder ;
		while(RHall == 0){
			Packer.Cpu1.State = Cpu1State_Error ;
			AccessCpu0(Cpu0Command_ShowCPU1ErrorCode);						
			Delay(800);
		}
		Delay(1000);
		Packer.ErrorCode = ErrorCode_NoError ;
		PowderLocate();
	}
	Counter = Packer.Cpu1.PowderLocation * 2;
	Packer.Cpu1.State = Cpu1State_Idel ;
	AccessCpu0(Cpu0Command_ShowSystemReady);
	Packer.Cpu1.Message = Message_SystemReady ;
}


//================== ADC0 P0.4     =======================================
void ADC0_Init (void)
{
   REF0CN = 0x0B;                      // Enable on-chip VREF and buffer

   AMX0P = 0x13;                       // ADC0 positive input = P0.4
   AMX0N = 0x1F;                       // ADC0 negative input = GND
                                       // i.e., single ended mode

   ADC0CF = 0xF8;

   EIE1 |= 0x08;                       // enable ADC0 conversion complete int.

   AD0EN = 1;                          // enable ADC0
   AD0BUSY = 1;
}
unsigned int meanCount = 100; // Total number of bytes to read from host
unsigned long accumulator = 0;     // Accumulator for averaging
//unsigned long  mV;    // Measured voltage in mV
unsigned int Tempture;
//unsigned int result=0;
//unsigned int ADC0ADD= 0;
void ADC0_ISR (void) interrupt 10
{
   AD0INT = 0;                               // Clear ADC0 conv. complete flag

   accumulator += ADC0L;
   accumulator = accumulator + ((ADC0H & 0x03) <<8);
//   accumulator += ADC0;
   meanCount --;

//	ADC0ADD = ADC0;

   if(meanCount==0)
   {  
      meanCount = 300; 
	  Tempture = accumulator /meanCount / 5.265 + 25 ; 
	  accumulator=0;
   }

}
//-----------------------------------------------------------------------------
void ShowPackType()
{
	if (Packer.Cpu0.PackType == PackType_Powder)
	{
		AccessCpu0(Cpu0Command_ShowPackPowder);
		Packer.Cpu1.Message = Message_PackPowder;
	}		
	else if (Packer.Cpu0.PackType == PackType_Drug)
	{
		AccessCpu0(Cpu0Command_ShowPackDrug);
		Packer.Cpu1.Message = Message_PackDrug;
	}
	else if (Packer.Cpu0.PackType == PackType_Mix)
	{
		AccessCpu0(Cpu0Command_ShowPackMix);
		Packer.Cpu1.Message = Message_PackMix;
	}
}
//-----------------------------------------------------------------------------
void ShowPackTypeUSB()
{
	if (USB.PackType == PackType_Powder)
	{
		AccessCpu0(Cpu0Command_ShowPackPowder);
		Packer.Cpu1.Message = Message_PackPowder;
	}		
	else if (USB.PackType == PackType_Drug)
	{
		AccessCpu0(Cpu0Command_ShowPackDrug);
		Packer.Cpu1.Message = Message_PackDrug;
	}
	else if (USB.PackType == PackType_Mix)
	{
		AccessCpu0(Cpu0Command_ShowPackMix);
		Packer.Cpu1.Message = Message_PackMix;
	}
}
//============================================================
void UART1_Init (void){
/*
	SBRLL1    = 0x3C;
    SBRLH1    = 0xF6;
    SCON1     = 0x10;		//埰勍UART1諉彶
    SMOD1     = 0x0E;
    SBCON1    = 0x43;  
*/
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
	if(cData == Packer.ErrorCode){
		EIE2 |= 0x02;
		SCON1&=0x3F;
	}
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

	u15 = u15PC_PaperPressOFF ;
	Delay(10);
//	u15 = u15PC_HeaterControlL	;
	u15 = u15PC_SharkerOFF ;
	Delay(10);
	u15 = u15PC_PaperResetH ;
	Delay(10);
	u15 = u15PC_PaperDisable;
}
//-----------------------------------------------------------------------------
void RxPackInfo()
{
	xdata	byte *i,aa[39],j=0;

	EIE2 &= ~0x02;

	for ( i = &Packer.Cpu0.PackType ; i <= (&Packer.Cpu0.PackType +38); i ++)
	{	
		while ( (SCON1 & 0x01) == 0);
		SCON1 &= (~0x01);	// RI=0
		*(i) = SBUF1;
		aa[j] = *i;
		j++ ;
	}
	BitPackParameter = Packer.Cpu0.BitPackParameter ;
	EIE2 |= 0x02;
}
//-----------------------------------------------------------------------------

void AccessCPU0(ECpu1Command Command)
{
	unsigned int i=0;
//	Packer.Cpu1.PowderLocation = (PowderLocationCounter ) /2;
//	Packer.Cpu1.Thermometer = Tempture;								 
	putchar(UART_Mark) ;

	putchar(Command);

	putchar(Packer.Cpu1.State);

	putchar(Packer.Cpu1.NowNumber);

	putchar(Packer.Cpu1.PowderLocation);

	putchar(Packer.Cpu1.Thermometer++);

	putchar(Packer.ErrorCode);
		

	IsRxCommand =0;
	while(!IsRxCommand) 
	{
		if (i++ >= 40000)	
		{
			i=0;
			Pointer_Packer = &Packer.UART_Mark;
			Packer.UART_Mark = UART_NoMark ;
			Cpu1CommandIdel();
			return;
		}
	}

	if (Command == Cpu0Command_TxPackInfo)
	{
		RxPackInfo();
	}
	else 
	{
		Delay(50);
	}

	Pointer_Packer = &Packer.UART_Mark ;

	if (Packer.Cpu0.State == Cpu0State_SettingMode)
	{
		Delay(1000);
	}
	else if (Packer.Cpu0.State == Cpu0State_PackingMode)
	{
//		if (Packer.UART_Mark == UART_Mark)
		{
			if (Packer.Cpu1.Command_UART == Cpu1Command_PackPause)
			{
				IsPackPause = 1;
				Cpu1CommandIdel();
			}
			else if (Packer.Cpu1.Command_UART == Cpu1Command_PackStop ||
					 Packer.Cpu1.Command_UART == Cpu1Command_PackContinue )
			{
				IsPackPause = 0;
			}
			else if (Packer.Cpu1.Command_UART == Cpu1Command_Again)
			{
				IsAgain = 1;
			}
		}
	}
}
//-----------------------------------------------------------------------------
byte Cpu1Command()
{
	if (Packer.Cpu1.Command_UART != Cpu1Command_Idel )
	{
		return Packer.Cpu1.Command_UART;
	}
	else 
	{
		return Packer.Cpu1.Command_USB;
	}
}
//-----------------------------------------------------------------------------
void Cpu1CommandIdel()
{
	Packer.Cpu1.Command_UART = Cpu1Command_Idel ;
	Packer.Cpu1.Command_USB  = Cpu1Command_Idel ;
}
//-----------------------------------------------------------------------------

void UART1_ISR(void)   interrupt 16 
{
	EIE2 &= ~0x02;
	if (SCON1 & 0x02)		// TI1
	{
		SCON1 &= (~0x02);
	}

	if (SCON1 & 0x01 )		// RI1
	{
		SCON1 &= (~0x01);
		*(Pointer_Packer) = SBUF1;
		if(Pointer_Packer == &Packer.UART_Mark)
		{
			if (Packer.UART_Mark == UART_Mark)
			{
				Packer.UART_Mark = UART_NoMark ;
				Pointer_Packer = & Packer.Cpu1.Command_UART ;

				RxTimeOutCount =0;
				TR2 =1;
			}
			else
			{	
				Pointer_Packer = &Packer.UART_Mark;
			}			
		}
		else if(Pointer_Packer == & Packer.Cpu1.Command_UART)
		{
			Pointer_Packer = &Packer.Cpu0.State ;
		}
		else if(Pointer_Packer == &Packer.Cpu0.State)
		{
			Pointer_Packer = &Packer.UART_Mark;	
			IsRxCommand = 1;
			RxTimeOutCount =0;
			TR2 =0;	
		}
		else 
		{
			Pointer_Packer = &Packer.UART_Mark;

		}

	}

	EIE2 |= 0x02;
}
//-----------------------------------------------------------------------------
void PowderCoverOpen()
{
	unsigned char i;
	i = Packer.Cpu1.PowderCoverOpenNumber ;
	if (i>4) i=2;

	u14 = u14PC_PowderCoverStart ;
	Delay(50);
	while(i--)
	{
		errorTime = Time_PowderCoverPS ;
		do 
		{
			i_Delay(Time_PSDelay);
			CheckErrorCode(ErrorCode_PowderCover);
		} while(PowderCoverPS==1) ;

		errorTime = Time_PowderCoverPS ;
		do 
		{
			i_Delay(Time_PSDelay);
			CheckErrorCode(ErrorCode_PowderCover);
		} while(PowderCoverPS==0) ;
	}
	u14 = u14PC_PowderCoverStop ;
	Delay(800);
}
//-----------------------------------------------------------------------------
void CheckErrorCode(EErrorCode ECode)
{
	if (errorTime-- == 0)
	{
		DisableAllDriver();

		Packer.ErrorCode = ECode ;
		while(1)
		{
			Packer.Cpu1.State = Cpu1State_Error ;
			AccessCpu0(Cpu0Command_ShowCPU1ErrorCode);	
			//AccessUSB(USBCommand_Busy);					
			Delay(800);
		}
	}
}
//-----------------------------------------------------------------------------
void AutoClear()
{
	Packer.Cpu1.PowderLocation =0;	
	Packer.Cpu1.State = Cpu1State_Clearing;

	AccessCpu0(Cpu0Command_ShowClearing);			
//	AccessUSB(USBCommand_Busy);	

	if (PaperMotorPS != 1)
	{
		PaperMotorLocate();		
	}
	Packer.Cpu1.VacuumType = VacuumType_HighSpeed ;
	Delay(10);
	PowderToLeft();


	CursorLocation = CursorRightEntry ;
	PowderCounterPS_LastState = PowderCounterPS ;

	PowderLocationCounter = 0;
	PowderLocate();

	Packer.Cpu1.VacuumType = VacuumType_Stop ;
	Delay(10);
	Packer.Cpu1.State = Cpu1State_Idel ;
	AccessCpu0(Cpu0Command_ShowSystemReady);
//	AccessUSB(USBCommand_Idel);	
}
//-----------------------------------------------------------------------------

void HeadWithPrint()
{
}