#include <C8051F340.h>
#include <a0.c>
//-----------------------------------------------------------------------------
void _Print(unsigned int Line)
{
	unsigned char i;

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

	if (Line < Right || Line >= Left)
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
	
		for (i=0 ; i < 21 ; i++)
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}
		for (i=0 ; i < 9 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0;
		}

		for (i=0 ; i < 8 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0x00;
		}

		for (i=0 ; i < 4 ; i++)	
		{
			while(!TXBMT);
			SPIF=0;
			SPI0DAT = 0;
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
 			SPI0DAT = 0x00;
		}
		for(i=0;i<6;i++){
			 while(!TXBMT);
			SPIF=0;
			if (Line>180&&Line<324) SPI0DAT = BB[Line-180][i];
			else 		SPI0DAT = 0x00;
		}

		for (i=0 ; i < 6; i++)
		{
			while(!TXBMT);
			SPIF=0;
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

	EA=1;
	EIE1 = tmpEIE1 ;
	EIE2 = tmpEIE2 ;
	FiveStep();
}
//-----------------------------------------------------------------------------