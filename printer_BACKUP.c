#include <C8051F340.h>
#include <communication2.h>
//#include <CPU1.h>
#include <intrins.h>
#include <a0.c>

void INIT_USB(void);
unsigned char Usb_StateA(void);
unsigned char Usb_StateB(void);
unsigned char Usb_Send(unsigned char PowderLocation);
void PCA0_Init (void);
void SPI_Init();
void OSCILLATOR_Init (void);
void Powder(unsigned char type,unsigned char pwm);
void _SET8255(unsigned char MOD);
extern xdata unsigned char memA[520][55] ;	
void _OnePack();
void _u14pc(unsigned char MOD);
void PresureUp();
void PresureDown();
void Delay(unsigned int i);
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

extern xdata unsigned char u14	;
idata signed char PowderLocationCounter;
idata unsigned char CursorLocation ;	
idata unsigned char PowderCounter ;
idata unsigned char PowderCounterPS_LastState ;
idata unsigned char PowderMotorState ;

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
void Cuter(void){
		P2 = P2 & 0x6F;
		P4 = 0xFF;
		_RESET = 0;
		_RD = 1;
		_WR	= 0;
		_A1=0;
		_A0=1;	
}
void DisableAllDriver(){
	unsigned int i=800;
	P2=0x7F;
	P4=0x00;
	_RESET = 0;
	_RD=1;
	_WR=0;
	_A1=1;_A0=0;
	_WR=1;		
	while(i--);
	P2=0x6F;
	P4=0x00;
	_RESET = 0;
	_RD=1;
	_WR=0;
	_A1=1;_A0=0;
	_WR=1;

}
void ini_u14(){
	unsigned int d=600;
	P2=0x6F;
	_RESET = 1;
	_RESET = 0;
	P4=0x90;
	_RD=1;
	_WR=0;
	_A1=1;_A0=1;
	_WR=1;
	while(d--);
	d=600;
	P4=0x00;
	_A1=1;_A0=0;
	_WR=0;
	while(d--);
	_WR=1;
}
void ini_u15(){
	P2=0x7F;
	_RESET = 1;
	_RESET = 0;
	P4=0x92;
	_RD=1;
	_WR=0;
	_A1=1;_A0=1;
	_WR=1;
	P4=0x00;
	_A0=0;
	P2=0x6F;
}

unsigned char i=0;
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
	Delay(5);
	_WR=1;
}
void _u14pc(unsigned char MOD){
	P2=0x6F;
	P4=MOD;
	_RESET = 0;
	_A1=1;_A0=0;
	_RD=1;
	_WR=0;
	Delay(5);
	_WR=1;	
}
void _u15pc(unsigned char MOD){
	P2=0x7F;
	P4=MOD;
	_RESET = 0;
	_RD=1;
	_WR=0;
	_A1=1;_A0=0;
	_WR=1;		
}
//-----------------------------------------------------------------------------
void _Print(unsigned int Line)
{
	unsigned char i,d;

	#define Right 	00
	#define Left  	520
	//temp = cA;
	unsigned char tmpEIE1;
	unsigned char tmpEIE2;

	unsigned int Shift = 0 ;

	tmpEIE1 = EIE1;
	tmpEIE2 = EIE2;
	EA =0;
	EIE1 = 0;
	EIE2 = 0;	
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
			if(i>60&&i<70)
				SPI0DAT = 0xF0;
			else	
				SPI0DAT = 0x00;
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
void main()
{   
	unsigned int d=600,x=2;
    char input_char;
//	unsigned char XX = 0xEB;
	Port_Ini();
	UART1_Init();
	SPI_Init();
	ini_u15();
	INIT_USB();
	DisableAllDriver();
    OSCILLATOR_Init ();                 // Initialize oscillator
    PCA0_Init ();                       // Initialize PCA0
    ini_u14();	
	Timer0_Init();
//	Interrupts_Init();
	_u14pb(0xFF);
//	_u14PAR();
//	_u14pc(0x01);
//	Usb_Send(1);

	while(1){
//	    input_char = _getkey();
//		WriteBack();
		if(Usb_StateA()){			
			//PowderLocate();
			Usb_Send(5);
//			Usb_Send();
//			u14 = u14PC_PresureStop;
//			u14 = u14PC_PowderRight;
//			PresureDown();
		}
		if(Usb_StateB()){
			Usb_Send(8);		
//			u14 = u14PC_PowderStop;
//			u14 = u14PC_PowderLeft;
		}

			
//		if(input_char == 0x01)
//			u14 = u14PC_CarbonStart;
//			putchar('C');
//		if(input_char == 0x05)
//			putchar(0x10);	
	}
}
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
//	_u15PAR();

	do 
	{
		Delay(1000);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS1==1) ;
//	errorTime = Time_PresurePS ;

	do 
	{
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS1==0) ;

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
	} while(PresurePS2==1);//while(P4==0x20) ;
//	errorTime = Time_PresurePS ;

	do 
	{
		Delay(1000);
//		i_Delay(Time_PSDelay);
//		CheckErrorCode(ErrorCode_PresureDown);	
	} while(PresurePS2==0);//while(P4==0x00) ;

	u14 = u14PC_PresureStop;
	Delay(800);
}

//-----------------------------------------------------------------------------
void TPHControl()
{
	unsigned char d;
	_u14pb(0xEF);	
	d=100;while(d--);
	_u14pb(0xFF);	

	d = 160;	while(d--);	
	// step 1	

	_u15pc(0xE0);	
		
	_u14pb(0xFC);
	
	d=2000;while(d--); 

	_u14pb(0xFF);	
	d = 10;	while(d--);	
	// step 2
	_u15pc(0xF0);		

	_u14pb(0xF3);	

	d=2000;while(d--);
  
	_u14pb(0xFF);
}
//-----------------------------------------------------------------------------
void FiveStep()
{
	unsigned int d;
	_u15pc(0xE0);		
	d = 700;	while(d--);
	_u15pc(0xF0);		
	d = 700;	while(d--);
	_u15pc(0xE0);		
	d = 700;	while(d--);
	_u15pc(0xF0);		
	d = 700;	while(d--);
	_u15pc(0xE0);		
	d = 700;	while(d--);
}
//-----------------------------------------------------------------------------
void _OnePack() 
{
	unsigned int n;	
	PresureUp();
	Carbon(1);
	PCA0CPH0 = 0x60;
	for (n=0;n< 640 ;n++)
	{

		_Print(n);
	}
	Carbon(0);
	PresureDown();
	_u15pc(0x00);
//	ClosePrinter();
	
}
//-----------------------------------------------------------------------------
void PaperMotorForward()
{
	unsigned int i;
	P4=0xE0;

	for (i=0 ; i < 80 ; i++)
	{
		//OneStep(800);	
		FiveStep();
	}

	_u15pc(0x00);
}
//-----------------------------------------------------------------------------
void PaperMotorReverse()
{
	unsigned int i;
	P4=0xC0;
	for (i=0 ; i < 80 ; i++)
	{
		OneStep(800);	
	}
	_u15pc(0x00);
}
//-----------------------------------------------------------------------------
void Delay(unsigned int i)
{
//	i=i<<1;
	while(i>0)
	{
		unsigned char d=100;	// 1mSec
		while(d--);
		/*
		while (d--)
		{
			while(!TF0);
			TF0 =0;
		}*/
		i-=1;
	}
}

//-----------------------------------------------------------------------------
void PCA0_Init (void)
{
   // Configure PCA time base; overflow interrupt disabled
   PCA0CN = 0x00;                      // Stop counter; clear all flags
   PCA0MD = 0x08;                      // Use SYSCLK as time base

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
}
//-----------------------------------------------------------------------------
void OneStep(unsigned int time)
{
	unsigned int d ;

	_u15pc(P4&0xE0);		
	d = time;	while(d--);

	_u15pc(P4|0x10);		
	d = time;	while(d--);

	_u15pc(P4&0xE0);		
	d = time;	while(d--);

	_u15pc(P4|0x10);	
	d = time;	while(d--);
	
	_u15pc(P4&0xE0);	
	d = time;	while(d--);

	_u15pc(P4|0x10);	
	d = time;	while(d--);
	
	_u15pc(P4&0xE0);	
	d = time;	while(d--);

	_u15pc(P4|0x10);		
	d = time-40;	while(d--);
}
//-----------------------------------------------------------------------------
void SPI_Init()
{
    SPI0CFG   = 0x40;
    SPI0CN    = 0x01;
    SPI0CKR   = 0x4A;
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
	switch(u14){
		case 0x00:
//			sMotor &= 0xFE;
			_u14pc(0x00);//Carbon stop
			break;
		case 0x01:
//			sMotor |= 0x01;
			_u14pc(0x01);//Carbon start
			break;
		case 0x02:
			_u14pc(0x00);
			break;
		case 0x03:
			_u14pc(0x02);
			break;
		case 0x04:
			_u14pc(0x00);
			break;
		case 0x05:
			_u14pc(0x04);
			break;
		case 0x06:
			_u14pc(0x00);
			break;
		case 0x07:
			_u14pc(0x08);
			break;
		case 0x08:
			PCA0CPH0=0xA0;
			_u14pc(0x20);
			break;
		case 0x09:
			PCA0CPH0=0xA0;
			_u14pc(0x30);
			break;
		case 0x0A:
			_u14pc(0x00);
			break;
		case 0x0B:
			break;
		case 0x0C:
			break;
		case 0x0D:
			break;
		case 0x0E:
			break;
		case 0x0F:
			break;
	}
//	_u14pc(sMotor);

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
	u14=0xff;
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
