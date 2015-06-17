//                     UPDATED 15.04.02 Good working version
//*****************************************************************************
// Program:	 Dimmer
// Author:	 Francis Lee
// Updated:	 February 28th, 2015
//*****************************************************************************
//#ifdef DebugOn
#define timerVal	2

//*****************************************************************************
// Firmware for Wireless/Touchless Dimmer
// Description: 
//
//*****************************************************************************

//***** PREPROCESSOR DIRECTIVES ***********************************************
 // INCLUDED FILES...
	#include	<ML610111.H>	// Lapis Micro ML610Q111 on LAPIS Development Board
	//#include	<ctype.h>		// Character classification and conversion 
	//#include	<errno.h>		// Error identifiers Library
	//#include	<float.h>		// Numerical limits for floating-point numbers
	//#include	<limits.h>		// Numerical limits for integers
	//#include	<math.h>		// Mathematical functions
	//#include	<muldivu8.h>	// Multiplication and Division accelerator
	//#include	<setjmp.h>		// Global jump (longjmp)
	//#include	<signal.h>		// Signal handling functions
	//#include	<stdarg.h>		// Variable numbers of arguments
	//#include	<stddef.h>		// Standard types and macros 
	#include	<stdio.h>		// I/O-related processing
	#include	<stdlib.h>		// General-purpose utilities
	//#include	<string.h>		// Character string manipulation routines
	//#include	<yfuns.h>		// 
	//#include	<yvals.h>		// Called for by most Header Files
	#include 	<uart.h>		// UART Function Prototypes
	#include 	<common.h>		// Common Definitions
	#include 	<mcu.h>		// MCU Definition
	#include 	<irq.h>		// IRQ Definitions
	#include	<i2c.h>		// I2C Definition
	//#include 	<main.h>		// Clear WDT API
	//#include 	<clock.h>		// Set System Clock API
	//#include 	<tbc.h>		// Set TBC (Timer Based Clock) API
	#include 	<timer.h>		// Timer Macros & APIs
//*****************************************************************************


//*****************************************************************************
//===========================================================================
//   MACROS: 
//===========================================================================
// ===== Serial Strings =====
#define PRINTF(msg)		write(0, msg, sizeof(msg))

// ===== Peripheral setting.=====
//#define HSCLK_KHZ	( 8000u )	// 8MHz = 8000kHz (will be multiplied by 1024 to give 8,192,000Hz)
//#define FLG_SET	( 0x01u )  
	#define HSCLK_KHZ	( 8192u )	// 8MHz = 8000kHz (will be multiplied by 1024 to give 8,192,000Hz)
	#define FLG_SET	    ( 0x01u ) 	

// ===== SET DESIRED UART SETTINGS HERE! (Options in UART.h) =====
	#define UART_BAUDRATE		( UART_BR_19200BPS) 	// Data Bits Per Second - Tested at rates from 2400bps to 512000bps!
	#define UART_DATA_LENGTH	( UART_LG_8BIT )		// x-Bit Data
	#define UART_PARITY_BIT		( UART_PT_NON )		// Parity
	#define UART_STOP_BIT		( UART_STP_1BIT )		// x Stop-Bits
	#define UART_LOGIC			( UART_NEG_POS )		// Desired Logic
	#define UART_DIRECTION		( UART_DIR_LSB )		// LSB or MSB First

//*****************************************************************************

//*****************************************************************************
//===========================================================================
//   STRUCTURES: 
//===========================================================================
static const tUartSetParam  _uartSetParam = {		// UART Parameters
	UART_BAUDRATE,						// Members of Structure...
	UART_DATA_LENGTH,						// Members of Structure...
	UART_PARITY_BIT,						// Members of Structure...
	UART_STOP_BIT,						// Members of Structure...
	UART_LOGIC,							// Members of Structure...
	UART_DIRECTION						// Members of Structure...
};

//*****************************************************************************


//*****************************************************************************
//===========================================================================
//   FUNCTION PROTOTYPES: 
//	Establishes the name and return type of a function and may specify the 
// 	types, formal parameter names and number of arguments to the function                                 
//===========================================================================
void main_clrWDT( void );			// no return value and no arguments
void Initialization( void );			// no return value and no arguments
void SetOSC( void );				// no return value and no arguments
//void analog_comparator( void );		// no return value and no arguments
void PortA_Low( void );				// no return value and no arguments
void PortB_Low( void );				// no return value and no arguments
void PortC_Low( void );				// no return value and no arguments
//void PortA_Digital_Inputs( void );		// no return value and no arguments
//void PinB0_PWM( void ); 			// no return value and no arguments

int write(int handle, unsigned char *buffer, unsigned int len);
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size);
void Init_KMX61( void );

void _funcUartFin( unsigned int size, unsigned char errStat );
void _funcI2CFin( unsigned int size, unsigned char errStat );
void checkI2C( void );
void main_reqNotHalt( void );
void _intUart( void );
void _intI2c( void );
void _intPB2( void );
void NOP1000( void );
void NOPms( unsigned int ms );
void OutputPWM(void);
void BLINK();
void BLINK_SLOW(void);
void SWEEP_RGB(void);

//*****************************************************************************

//GLOBALS...
unsigned char	_flgUartFin;
unsigned char 	_flgI2CFin;
unsigned char	_flgPB2Int;
unsigned char	_reqNotHalt;
unsigned char	RegVal;
float			SensorReturn;
unsigned char DoorOpen = 0;
unsigned int TimeFlag = timerVal; // 2343 = 5minutes / 128ms (length of existing timer)
							 // 938 = 2min
							 // 1875 = 4minutes / 128ms
unsigned int timer;


union {
	unsigned char	_uchar;
	unsigned char	_ucharArr[6];
	unsigned int	_uint;
	unsigned int	_uintArr[3];
	int				_intArr[3];
	float			_float;
} uniRawSensorOut;


const unsigned char Manufact_ID			= 0x92u;
const unsigned char Prox_ModeCTR		= 0x41u; 

const unsigned char Prox_PS_LSB			= 0x44u; 
const unsigned char Prox_PS_MSB			= 0x45u; 
const unsigned char ALS0_LSB			= 0x46u; 
const unsigned char ALS0_MSB			= 0x47u;  
const unsigned char ALS1_LSB			= 0x48u; 
const unsigned char ALS1_MSB			= 0x49u;  
/**
 * KMX61 (Digital Tri-axis Magnetometer/Tri-axis Accelerometer)
 */
// I2C device address of KMX61
const unsigned char KMX61_I2C_ADDR			= 0x0eu;
// Register addresses of KMX61
const unsigned char KMX61_WHO_AM_I			= 0x00u;
const unsigned char KMX61_INS1				= 0x01u;
const unsigned char KMX61_INS2				= 0x02u;
const unsigned char KMX61_STATUS_REG		= 0x03u;
const unsigned char KMX61_ACCEL_XOUT_L		= 0x0au;
const unsigned char KMX61_ACCEL_XOUT_H		= 0x0bu;
const unsigned char KMX61_ACCEL_YOUT_L		= 0x0cu;
const unsigned char KMX61_ACCEL_YOUT_H		= 0x0du;
const unsigned char KMX61_ACCEL_ZOUT_L		= 0x0eu;
const unsigned char KMX61_ACCEL_ZOUT_H		= 0x0fu;
const unsigned char KMX61_TEMP_OUT_L		= 0x10u;
const unsigned char KMX61_TEMP_OUT_H		= 0x11u;
const unsigned char KMX61_MAG_XOUT_L		= 0x12u;
const unsigned char KMX61_MAG_XOUT_H		= 0x13u;
const unsigned char KMX61_MAG_YOUT_L		= 0x14u;
const unsigned char KMX61_MAG_YOUT_H		= 0x15u;
const unsigned char KMX61_MAG_ZOUT_L		= 0x16u;
const unsigned char KMX61_MAG_ZOUT_H		= 0x17u;
const unsigned char KMX61_XOUT_HPF_L		= 0x18u;
const unsigned char KMX61_XOUT_HPF_H		= 0x19u;
const unsigned char KMX61_YOUT_HPF_L		= 0x1au;
const unsigned char KMX61_YOUT_HPF_H		= 0x1bu;
const unsigned char KMX61_ZOUT_HPF_L		= 0x1cu;
const unsigned char KMX61_ZOUT_HPF_H		= 0x1du;
const unsigned char KMX61_SN_1				= 0x24u;
const unsigned char KMX61_SN_2				= 0x25u;
const unsigned char KMX61_SN_3				= 0x26u;
const unsigned char KMX61_SN_4				= 0x27u;
const unsigned char KMX61_INL				= 0x28u;
const unsigned char KMX61_STBY_REG			= 0x29u;
const unsigned char KMX61_CNTL1				= 0x2au;
const unsigned char KMX61_CNTL2				= 0x2bu;
const unsigned char KMX61_ODCNTL			= 0x2cu;
const unsigned char KMX61_INC1				= 0x2du;
const unsigned char KMX61_INC2				= 0x2eu;
const unsigned char KMX61_INC3				= 0x2fu;
const unsigned char KMX61_COTR				= 0x3cu;
const unsigned char KMX61_WUFTH				= 0x3du;
const unsigned char KMX61_WUFC				= 0x3eu;
const unsigned char KMX61_BTH				= 0x3fu;
const unsigned char KMX61_BTSC				= 0x40u;
const unsigned char KMX61_TEMP_EN_CNTL		= 0x4cu;
const unsigned char KMX61_SELF_TEST			= 0x60u;
const unsigned char KMX61_BUF_THRESH_H		= 0x76u;
const unsigned char KMX61_BUF_THRESH_L		= 0x77u;
const unsigned char KMX61_BUF_CTRL1			= 0x78u;
const unsigned char KMX61_BUF_CTRL2			= 0x79u;
const unsigned char KMX61_BUF_CLEAR			= 0x7au;
const unsigned char KMX61_BUF_STATUS_REG	= 0x7bu;
const unsigned char KMX61_BUF_STATUS_H		= 0x7cu;
const unsigned char KMX61_BUF_STATUS_L		= 0x7du;
const unsigned char KMX61_BUF_READ			= 0x7eu;
// Configuration data
// Disable self-test function
const unsigned char KMX61_SELF_TEST_CFGDAT = 0x0u;
// Disable Back to Sleep engine, Wake up engine and interrupt pin.
// Set operating mode is higher power mode and +/-8g - 14bit
const unsigned char KMX61_CNTL1_CFGDAT	= 0x53u;	//Also added WUFE Bit
// Set Output Data Rate at which the wake up (motion detection) is 0.781Hz
const unsigned char KMX61_CNTL2_CFGDAT	= 0x0u;
// Set Output Data Rate of accelerometer and magnetometer are 12.5Hz
const unsigned char KMX61_ODCNTL_CFGDAT	= 0x0u;
// Enable the Temperature output when the Magnetometer is on
const unsigned char KMX61_TEMP_EN_CNTL_CFGDAT = 0x01u;
// Set operating mode of the sample buffer is FIFO
const unsigned char KMX61_BUF_CTRL1_CFGDAT = 0x0u;
const unsigned char KMX61_BUF_CTRL2_CFGDAT = 0x0u;
const unsigned char KMX61_INC1_CFGDAT = 0x29u;
const unsigned char KMX61_INC3_CFGDAT = 0x02u;
const unsigned char KMX61_WUFTH_CFGDAT = 0x08;
const unsigned char KMX61_WUFC_CFGDAT = 0x01;


static unsigned char SensorPlatformSelection;
static unsigned char SensorIntializationFlag = 1;
static unsigned char LEDFlashFlag = 0;
static unsigned char LEDChangeFlag = 0;


static unsigned char			val[800];
static unsigned char			buffer[20]; 
static unsigned char			word[100]; 
static unsigned char            temp;
static unsigned int             flag;
static unsigned int             checkSum;
static unsigned int             wordIndex;
static unsigned int             hexToDecOffset; 
static unsigned char			str[8] = {0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};//str[8] = {0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09};

static unsigned int             UTC[3];
static unsigned char            LatDir, LonDir,LatLonValid, Mode[2];
static unsigned  int 			i,SV_ID[12];
static unsigned  int 			j, sigDigits;
static long double 				Latitude, Longitude,HDOP,PDOP,VDOP,MSL,Geoid;
static unsigned int 			fixQuality,numSat,isNeg;
static unsigned int				GSV_numMessage,GSV_index,numSat,PRN_num,Eleveation,Azimuth,SNR,GSV_Info[12];

static long double 				GroundSpeed,Course,MagneticVariation;
static unsigned char            ReceiverState,MagDir;
static unsigned int             Date[2];

static long double				TrueDegrees,MagDegrees,KnotsSpeed,GroundSpeed;
static unsigned char			isTrueNorth,isMagNorth,speedUnit,KnotsUnit ,KMHSpeed;

static unsigned int				CheckPointIndex,isSettingMode;
static long double				LatDest[5],LonDest[5],HomeTolerance;


static long double				prevBulbIntensity;
static unsigned int             bulbIntensity, bulbEnable;
static unsigned char			txChar[3],isBulbOn;
/*############################################################################*/
/*#                                  APIs                                    #*/
/*############################################################################*/
//===========================================================================
//  	Start of MAIN FUNCTION
//===========================================================================
int main(void) 
{ 	 
	Initialization(); //Ports, UART, Timers, Oscillator, Comparators, etc.
	temp = 0xC6u; //1100_0110
	I2C_Write(0x38u, &Prox_ModeCTR, 1, &temp, 1); 
	main_clrWDT(); 
	I2C_Read(0x38u, &Manufact_ID, 1, uniRawSensorOut._ucharArr, 1); 
	main_clrWDT(); 
	I2C_Read(0x38u, &Prox_ModeCTR, 1, uniRawSensorOut._ucharArr, 1);
	
		main_clrWDT(); 
		PB4D = 1;
		PA1D = 0;
		PC2D = 1;
		NOPms(20);
		for(i=0;i<100;i++){
			main_clrWDT();  
			NOPms(100); 
		}
		PB4D = 0;
		PA1D = 1;
		PC2D = 1;
		NOPms(20);
		
		while(flag < 30){
			main_clrWDT(); 
			I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
			flag = (int)uniRawSensorOut._ucharArr[0];
			NOPms(20);
		}
     
		main_clrWDT(); 
		PB4D = 0;
		PA1D = 1;
		PC2D = 1;
		NOPms(20);
		
		
		PB4D = 1;
		PA1D = 0;
		PC2D = 1;
		NOPms(20);
		
		PB4D = 1;
		PA1D = 1;
		PC2D = 0;
		NOPms(20);
	 
		PB4D = 1;
		PA1D = 1;
		PC2D = 1; 
		  
						
						
	txChar[0] = 1;
	txChar[1] = 1;
	txChar[2] = 0;
	main_clrWDT();  
	_flgUartFin = 0;
	uart_stop();  
	uart_startSend(txChar, sizeof(txChar), _funcUartFin);
	while(_flgUartFin != 1){
		main_clrWDT(); 
	}   
	bulbIntensity = 0;
	i = 0;
	while(1){ 
			main_clrWDT(); 
			I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
			main_clrWDT(); 
			if((int)uniRawSensorOut._ucharArr[0] > 10){
				i = 0;
				BLINK();
				main_clrWDT();  
				
			    while((int)uniRawSensorOut._ucharArr[0] > 5){ 
					main_clrWDT(); 
					I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
				    BLINK(); 
					i++; 
					if(i>50)
						uniRawSensorOut._ucharArr[0] = 0;
				}  
				if(i>50){
					BLINK_SLOW(); 
					
					txChar[0] = 'c'; 
					main_clrWDT();  
					_flgUartFin = 0;
					uart_stop();  
					uart_startSend(txChar, sizeof(txChar), _funcUartFin);
					while(_flgUartFin != 1){
						main_clrWDT(); 
					}   
					
					i=0;
					main_clrWDT(); 
					I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
					while((int)uniRawSensorOut._ucharArr[0] < 10){
						main_clrWDT(); 
						I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
						
						main_clrWDT(); 
						PB4D = 0;
						PA1D = 1;
						PC2D = 1;
						NOPms(20);
						
						
						PB4D = 1;
						PA1D = 0;
						PC2D = 1;
						NOPms(20);
						
						PB4D = 1;
						PA1D = 1;
						PC2D = 0;
						NOPms(20);
					 
						PB4D = 1;
						PA1D = 1;
						PC2D = 1; 
		
						NOPms(100); 
					}
					
					txChar[0] = 'd';
					main_clrWDT();  
					_flgUartFin = 0;
					uart_stop();  
					uart_startSend(txChar, sizeof(txChar), _funcUartFin);
					while(_flgUartFin != 1){
						main_clrWDT(); 
					}   
						 
					BLINK_SLOW();   
				}else{   
					if(txChar[0] != 'a'){
						txChar[0] = 'a'; 
					}else{
						txChar[0] = 'b';  
					}
				}
				
				main_clrWDT();  
				_flgUartFin = 0;
				uart_stop();  
				uart_startSend(txChar, sizeof(txChar), _funcUartFin);
				while(_flgUartFin != 1){
					main_clrWDT(); 
				}   
			} 
			I2C_Read(0x38u, &Prox_PS_MSB, 1, uniRawSensorOut._ucharArr, 1);
			main_clrWDT(); 
			I2C_Read(0x38u, &ALS0_LSB, 1, uniRawSensorOut._ucharArr, 1);
			main_clrWDT(); 
			I2C_Read(0x38u, &ALS0_MSB, 1, uniRawSensorOut._ucharArr, 1);
			main_clrWDT(); 
			I2C_Read(0x38u, &ALS1_LSB, 1, uniRawSensorOut._ucharArr, 1);
			main_clrWDT(); 
			I2C_Read(0x38u, &ALS1_MSB, 1, uniRawSensorOut._ucharArr, 1); 
	}
	 
	
	//OutputPWM(); 
	isSettingMode = 0;
	bulbIntensity = 200; 
	bulbEnable = 1;
	while(1){ 
			main_clrWDT(); 
			I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
			flag = (int)uniRawSensorOut._ucharArr[0];
			  
			if(flag > 30){
			    NOPms(100);
			    NOPms(100);
			    NOPms(100);
				I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
				flag = (int)uniRawSensorOut._ucharArr[0];
				
				if(flag < 30){
					bulbEnable = !bulbEnable; 
					main_clrWDT();
				}
				else{
					while(flag > 30){  
						for(i=1; i< 250 && flag >30; i++){
							main_clrWDT();
							PCRUN = 0;
							PWCD = i;
							PCRUN = 1;
							I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
							flag = (int)uniRawSensorOut._ucharArr[0]; 
							NOPms(60); 
						
							i+=1;
						}  
						if(flag < 30){
							bulbIntensity = i;
							bulbEnable = 1;
							NOPms(100);
							break;
						}
						for(i=250; i> 1 && flag >30; i--){
							main_clrWDT();
							PCRUN = 0;
							PWCD = i;
							PCRUN = 1;
							I2C_Read(0x38u, &Prox_PS_LSB, 1, uniRawSensorOut._ucharArr, 1);
							flag = (int)uniRawSensorOut._ucharArr[0]; 
							NOPms(60);  
						} 
						if(flag < 30){
							bulbIntensity = i;
							bulbEnable = 1;
							NOPms(100);
							break;
						}  
					} 
				}
			}
			if(bulbEnable){
				PCRUN = 0;
				PWCD = bulbIntensity;
				PCRUN = 1;
				prevBulbIntensity = bulbIntensity;
			}
			else{
				PCRUN = 0;
				PWCD = 0; 
				//PCRUN = 1; 
				prevBulbIntensity = 0;
			}
			
			//GetUART_Command();
			
			//Adjust PWM to Safe Level just below the "motor on" point
			//PFRUN = 0;
			//PERUN = 0;
			//PDRUN = 0;
			//PCRUN = 0;
				
			//PWF0D = PWMSafeDuty; //Can't be running to change
			//PWED = PWMSafeDuty;
			//PWDD = PWMSafeDuty;
			//PWCD = 2000;//flag*4+50;
			
			//PFRUN = 1;
			//PERUN = 1;
			//PDRUN = 1;
			//PCRUN = 1;  
	}


	 
}


void SWEEP_RGB(){ 
}

//===========================================================================
//  	End of MAIN FUNCTION
//===========================================================================
void BLINK_SLOW(void){
		main_clrWDT(); 
		PB4D = 0;
		PA1D = 1;
		PC2D = 1;
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		
		PB4D = 1;
		PA1D = 0;
		PC2D = 1;
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		
		PB4D = 1;
		PA1D = 1;
		PC2D = 0;
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
	 
		PB4D = 1;
		PA1D = 1;
		PC2D = 1;
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
		NOPms(100);
}


void BLINK(void){
		main_clrWDT(); 
		PB4D = 0;
		PA1D = 1;
		PC2D = 1;
		NOPms(20);
		
		PB4D = 1;
		PA1D = 0;
		PC2D = 1;
		NOPms(20);
		
		PB4D = 1;
		PA1D = 1;
		PC2D = 0;
		NOPms(20);
	 
		PB4D = 1;
		PA1D = 1;
		PC2D = 1;
}

void GetUART_Command(void){
		val[0] = 0;
		uart_stop();
		uart_startReceive(val, 1, _funcUartFin); 
		prevBulbIntensity = (prevBulbIntensity * 1.22437);
		NOPms(10);  
		if(val[0]){
		    if(prevBulbIntensity > 230.0){
				write(0,"Bulb Fully On\n",sizeof("Bulb Fully On\n"));
			}
			else if(prevBulbIntensity == 0){
				write(0,"Bulb is Off\n",sizeof("Bulb is Off\n"));
			}
			else{
				//sprintf(val,"Intensity is %f\n", prevBulbIntensity);
				write(0,val,sizeof(val));
				//write(0,"Intensity is \n",sizeof("Intensity is \n"));
			}
		}
}

//===========================================================================
//  	End of MAIN FUNCTION
//===========================================================================
//*****************************************************************************




//*****************************************************************************
//===========================================================================
//  	Start of Other Functions...
//===========================================================================
//*****************************************************************************


/*******************************************************************************
	Routine Name:	main_clrWDT
	Form:			void main_clrWDT( void )
	Parameters:		void
	Return value:	void
	Description:	clear WDT.
******************************************************************************/

void main_clrWDT( void )
{
	do {
		WDTCON = 0x5Au;
	} while (WDP != 1);
	WDTCON = 0xA5u;
}

/*******************************************************************************
	Routine Name:	NOP1000
	Form:			void NOP( void )
	Parameters:		void
	Return value:	void
	Description:	NOP for 1000 Cycles.
******************************************************************************/
void NOP1000( void )
{
unsigned int ONCNT = 0;

	while(ONCNT < 1000) {	// NOP for 1000 Cycles
		ONCNT++;
	}
	ONCNT = 0;			// Reset Counter 
}

/*******************************************************************************
	Routine Name:	NOPms
	Form:			void NOP1000( unsigned int ms )
	Parameters:		unsigned int sec = "Number of seconds where the device is not doing anything"
	Return value:	void
	Description:	NOP for x seconds. Uses HTB* clock (512kHz) and timer 8+9 (max 0xFFFF)
					*(HTBCLK = 1/16 * HSCLK = (1/16)*8192kHz = 512kHz, see HTBDR to change if we need an even smaller increment timer...)
					1/(512kHz) * 0xFFFF = 127ms
					
					(HTBCLK = 1/16 * HSCLK = (1/16)*1024kHz = 64kHz, see HTBDR to change if we need an even smaller increment timer...)
					1/(512kHz) * 0xFFFF = 1.02secs
					
******************************************************************************/
void NOPms( unsigned int ms )
{
	unsigned int timerThres;
	unsigned char TimeFlag;
	unsigned int TempSec;
	unsigned int timer;
	unsigned int timertest;

	TempSec = ms;
	TimeFlag = 0;

	tm_init(TM_CH_NO_AB);
	tm_setABSource(TM_CS_HTBCLK);
	tm_setABData(0xffff);

	if(ms < 128){
		timerThres = 0x1FF * ms;
		TimeFlag = 0;
	}
	if(ms == 128){
		timerThres = 0xFFFF;
		TimeFlag = 0;
	}
	if(ms > 128){
		while(TempSec > 128){
			TempSec -= 128;
			TimeFlag++;
		}
		if(TempSec != 0){
			timerThres = 0x1FF * TempSec;
		}
		else{
			timerThres = 0xFFFF;
			TimeFlag--;
		}
	}

TimerRestart:
	main_clrWDT();	
	//tm_restart89();	//using LSCLK, the maximum delay time we have is ~2 secs
	tm_startAB();
	timer = tm_getABCounter();
	while(timer < timerThres){
		timer = tm_getABCounter();
		timertest = timer;
	}
	if(TimeFlag !=0){
		tm_stopAB();
		TimeFlag--;
		timerThres = 0xFFFF;
		goto TimerRestart;
	}
}


//===========================================================================
//	Initialize Micro to Desired State...
//===========================================================================
static void Initialization(void)
{

	//Initialize Peripherals	
	//BLKCON2 Control Bits...Manually Set 4/12/2013
	DSIO0 = 0; // 0=> Enables Synchronous Serial Port 0 (initial value).
 
	DUA0  = 0; // 0=> Enables the operation of UART0 (initial value). 
	 
	DUA1  = 1; // 0=> Enables Uart1 (initial value). 
	DI2C1 = 1; // 0=> Enables I2C bus Interface (Slave) (initial value).
	DI2C0 = 0; // 0=> Enables I2C bus Interface (Master) (initial value).	
	
	BLKCON4 = 0x01; // 0=> Enables SA-ADC
	BLKCON6 = 0xC3; // (1=disables; 0=enables) the operation of Timers 8, 9, A, E, F.
					// only timer AB are enabled
	BLKCON7 = 0x0B; // (1=disables; 0=enables) the operation of PWM (PWMC, PWMD, PWME, PWMF


	// Port Initialize
	PortA_Low();	//Initialize all 3 Ports of Q111 Port A to GPIO-Low
	PortB_Low();	//Initialize all 8 Ports of Q111 Port B to GPIO-Low
	PortC_Low();	//Initialize all 4 Ports of Q111 Port C to GPIO-Low

	// ===== Set Oscillator Rate =====
	SetOSC(); //8MHz
 
	// INTERRUPT SETUP...
	irq_di(); 	//Disable Interrupts...
	irq_init();	//Initialize Interrupts (All Off and NO Requests)
	
	// INTERRUPT ENABLE REGISTERS...
	IE0 = IE1 = IE2 = IE3 = IE4 = IE5 = IE6 = IE7 = 0; // 0=DISABLED; 1=ENABLED

	// INTERRUPT REQUEST REGISTERS...Used to request an interrupt from a selected interrupt source.
	IRQ0 = IRQ1 = IRQ2 = IRQ3 = IRQ4 = IRQ5 = IRQ6 = IRQ7 = 0;	//Clear all Requests...

	E2H = 0; 	// E2H is the Enable flag for 2Hz TBC Interrupt (1=ENABLED)
	 
	irq_setHdr((unsigned char)IRQ_NO_UA0INT, _intUart);
	EUA0 = 1; 	// EUA0 is the enable flag for the UART0 interrupt (1=ENABLED)
 
	 
	irq_setHdr((unsigned char)IRQ_NO_I2CMINT, _intI2c);
	EI2CM = 1;
	QI2CM = 0; 
	
	//Need to setup PB2 as external interrupt pin
	PB2MD0 = 0;
	PB2MD1 = 0;
	PB2DIR = 1;
	PB2C0 = 0;
	PB2C1 = 1;
	PB2E0 = 1;
	PB2E1 = 0;
	
	
	//Setup the Callback Function for External interrupt on PB2 for Accel
	irq_setHdr((unsigned char)IRQ_NO_PB2INT, _intPB2);
	EPB2 = 1;
	QPB2 = 0;
	
	irq_ei();	// Enable Interrupts...

	// WDT...
	WDTMOD = 0x03; 	// 0x03=overflow 8sec...
	main_clrWDT(); 	// Clear WDT

	//I2C Initialization...
	//P20C0 = 1;	// CMOS output 
	//P20C1 = 1;	
	//P20D = 1;	    // write protect enable 
	i2c_init(I2C_MOD_FST, (unsigned short)HSCLK_KHZ, I2C_SYN_OFF);
	
	//UART Initialization...
 
	(void)uart_init( (unsigned char)UART_CS_HSCLK,		// Generator        
			     (unsigned short)HSCLK_KHZ,				// HSCLK frequency  
			     &_uartSetParam );						// Param... 	 
	uart_PortSet();
	_flgUartFin = 0;
	uart_stop();
 
	
	//Initialize GPIO3 for Garage Controller
	PB4DIR = 0;		// PortB Bit7 set to Output Mode...
	PB4C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB4C0  = 1;	
	PB4MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB4MD0  = 0;
	PB4D = 0;		// B.7 Output OFF....
	 
}//End Initialization
//===========================================================================


void OutputPWM(void){ 
//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Select the Clock Mode...
//Step 5: Set the Duty Cycle...
//Step 5: Start the PWM Counter...

//The PWM signals with the periods of approximately 122 ns (@PLLCLK=16.384MHz) to 2s (@LSCLK=32.768kHz)
//  can be generated and output outside of this micro!

      //Direction...    
      PA2DIR = 0;       // PortB Bit0 set to Output Mode... 
	    
      //I/O Type...
      PA2C1  = 1;       // PortB Bit0 set to CMOS Output...
      PA2C0  = 1;       
	   
      //Purpose...
      PA2MD1  = 0;            // PortC Bit0 set to PWM Output (1,0)...
      PA2MD0  = 1;    
	 
      //Select the Clock Mode... 
      PECS1 = 0;        //00= LS; 01=HS; 10=PLL
      PECS0 = 1;

	  //PECS1 = 0;
	  //PECS0 = 1;
	    
      //SET THE PERIOD...(Added June 4th, 2013)
      PWEP = 27000;            // Init Period to (1=255kHz; 10=46kHz; 50=10kHz; 200=2.5kH; ; 3185 = 160Hz; 3400=150Hz; 4250=120Hz; 5000=102Hz)
 
      //SET THE DUTY CYCLE...(Added June 15th, 2013)

      //PWCD =    10;         //10    ~  0.2  % duty cycle @ 120Hz
      //PWCD =   100;         //100   ~  2.4  % duty cycle @ 120Hz
      //PWCD =  1000;         //1000  ~ 23.5  % duty cycle @ 120Hz
      //PWF0D =  10000;         //4000  ~ 94.0  % duty cycle @ 120Hz
	  PWED = 10000;
      //PWF1D =  10000;         // G
      //PWF2D =  10000;         // G
	  //PWED =  10000;          // R
      //PWCD =  4150;         //4150  ~ 99.0  % duty cycle @ 120Hz
      //PWCD =    20;         //20    ~  0.4  % duty cycle @ 120Hz      
      //PWF0D =    PWMSafeDuty;           //12    ~  0.25 % duty cycle @ 160Hz

      //PFRUN = 0;        // OFF to start
      PERUN = 0;        // OFF to start
}



/*******************************************************************************
	Routine Name:	Init_KMX61
	Form:			void Init_KMX61( void )
	Parameters:		void
	Return value:	void
	Initialization: None.
	Description:	Gets the output of Sensor of Sensor Control 16.
	Sensor Platform(s): 6-Axis Accelerometer/Magnetometer
						KMX61
******************************************************************************/
void Init_KMX61()
{ 
	// Set accelerometer and magnetometer to stand-by mode
	RegVal = 0x03u;
	I2C_Write(KMX61_I2C_ADDR, &KMX61_STBY_REG, 1, &RegVal, 1);
	// Configure
	I2C_Write(KMX61_I2C_ADDR, &KMX61_SELF_TEST, 1, &KMX61_SELF_TEST_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_CNTL1, 1, &KMX61_CNTL1_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_CNTL2, 1, &KMX61_CNTL2_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_ODCNTL, 1, &KMX61_ODCNTL_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_TEMP_EN_CNTL, 1, &KMX61_TEMP_EN_CNTL_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_BUF_CTRL1, 1, &KMX61_BUF_CTRL1_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_BUF_CTRL2, 1, &KMX61_BUF_CTRL2_CFGDAT, 1);
	
	//Configure Accel's Interrupt output - Interrupt on Wake-up...
	I2C_Write(KMX61_I2C_ADDR, &KMX61_INC1, 1, &KMX61_INC1_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_INC3, 1, &KMX61_INC3_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_WUFTH, 1, &KMX61_WUFTH_CFGDAT, 1);
	I2C_Write(KMX61_I2C_ADDR, &KMX61_WUFC, 1, &KMX61_WUFC_CFGDAT, 1);
	
	// Set accelerometer and magnetometer to operating mode
	RegVal = 0x02u;
	I2C_Write(KMX61_I2C_ADDR, &KMX61_STBY_REG, 1, &RegVal, 1);
}

/*******************************************************************************
	Routine Name	: write
	Form			: int write(int handle, unsigned char *buffer, unsigned int len)
	Parameters		: int handle
					  unsigned char *buffer
					  unsigned int len
	Return value	: int
	Initialization	: None.
	Description		: The write function writes len bytes of data from the area specified by buffer to UART0.
******************************************************************************/
int write(int handle, unsigned char *buffer, unsigned int len)
{
	_flgUartFin = 0; 
	uart_stop();
	uart_startSend(buffer, len, _funcUartFin); 
	while(_flgUartFin != 1)
	{
		main_clrWDT();
	}
	return len;
}

/*******************************************************************************
	Routine Name	: I2C_Read
	Form			: void I2C_Read(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Read(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startReceive(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name	: I2C_Write
	Form			: void I2C_Write(unsigned char slave_address, unsigned char *address, unsigned char address_size, unsigned char *buffer, unsigned char size)
	Parameters		: unsigned char slave_address
					  unsigned char *address
					  unsigned char address_size
					  unsigned char *buffer
					  unsigned char size
	Return value	: void
	Initialization	: None.
	Description		: 
******************************************************************************/
void I2C_Write(unsigned char slave_address, unsigned char *reg_address, unsigned char reg_address_size, unsigned char *buffer, unsigned char size)
{
	_flgI2CFin = 0;
	i2c_stop();	
	i2c_startSend(slave_address, reg_address, reg_address_size, buffer, size, (cbfI2c)_funcI2CFin);
	while(_flgI2CFin != 1)
	{
		main_clrWDT();
	}
}

/*******************************************************************************
	Routine Name:	_funcUartFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcUartFin( unsigned int size, unsigned char errStat )
{
	uart_continue();					// Function in UART.c: process to continue send and receive...
	_flgUartFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_funcI2CFin
	Form:			static void _funcUartFin( unsigned int size, unsigned char errStat )
	Parameters:		unsigned int size		 : 
				unsigned char errStat	 : 
	Return value:	void
	Description:	UART transmission completion callback function.
******************************************************************************/
static void _funcI2CFin( unsigned int size, unsigned char errStat )
{
	i2c_continue();					// Function in UART.c: process to continue send and receive...
	_flgI2CFin = (unsigned char)FLG_SET;
	main_reqNotHalt();				// uncommented 5/2/2013
}

/*******************************************************************************
	Routine Name:	_intI2c
	Form:			static void _intI2c( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intI2c( void )
{
	(void)i2c_continue();
	main_reqNotHalt();
}

/*******************************************************************************
	Routine Name:	_intPB2
	Form:			static void _intADC( void )
	Parameters:		void
	Return value:	void
	Description:	I2C handler.
******************************************************************************/
static void _intPB2( void )
{
	_flgPB2Int = 1;
	//PRINTF("PB2 Int Works!");
}

/*******************************************************************************
	Routine Name:	checkI2C
	Form:			void checkI2C( void )
	Parameters:		void
	Return value:	void
	Description:	Reading or writing processing of I2C Bus.
******************************************************************************/
void checkI2C( void )
{
int		ret;
	
	ret = 0;
	//P21C1 = 1;
	while (ret != 1) {
		ret = i2c_continue();
		if( ret == 1 ) {
			//P21C1 = 0;
		}
	}
}

/*******************************************************************************
	Routine Name:	main_reqNotHalt
	Form:			void reqNotHalt( void )
	Parameters:		void
	Return value:	void
	Description:	request not halt.
******************************************************************************/
void main_reqNotHalt( void )
{
	_reqNotHalt = (unsigned char)FLG_SET;
}

/*******************************************************************************
	Routine Name:	_intUart
	Form:			static void _intUart( void )
	Parameters:		void
	Return value:	void
	Description:	UART handler.
******************************************************************************/
static void _intUart( void )
{
		uart_continue(); //in UART.c: process to continue send and receive...
}

//===========================================================================
//	OSC set
//===========================================================================
static void SetOSC(void){
	//Frequency Control Register 0 (FCON0) 	
	//	used to control the high-speed clock generation circuit and to select system clock.
		SYSC0  = 0;			// Used to select the frequency of the HSCLK => 00=8.192MHz.
		SYSC1  = 0;			//...
		OSCM1  = 1;			// 10 => Built-in PLL oscillation mode
		OSCM0  = 0;			//...
		
	// Frequency Control Register 1 (FCON1)
		ENOSC  = 1;			// 1=Enable High Speed Oscillator...
		SYSCLK = 1;			// Select System Clock: 1=HSCLK; 0=LSCLK 
		LPLL   = 1;			// 1=Enables the use of PLL oscillation - ADDED 4/30/2013

	__EI();					//INT enable 
}
//===========================================================================


//===========================================================================
//	Analog Comparator setup
//===========================================================================
void analog_comparator(void){

//Carl's Notes...

//Step 1: Select the Interrupt Mode
// 	a.) Interrupt Disabled      => CMPxE1 = 0; CMPxE0 = 0;	  
// 	b.) Falling-Edge Int. Mode  => CMPxE1 = 0; CMPxE0 = 1;
// 	c.) Rising-Edge Int. Mode   => CMPxE1 = 1; CMPxE0 = 0;
// 	d.) Both-Edge Int. Mode     => CMPxE1 = 1; CMPxE0 = 1;


//Step 2: Enable the Comparator                       => CMPxEN = 1;	

//Step 3: Wait 3ms to allow Comparator to stabilize

//Step 4: Read the comparison result			=> CMPxD: 0= +<-; 1= +>-

//Step 5: Disable the Comparator				=> CMPxEN = 0;	



   //Comparator 0...
	CMP0EN  = 0x01; 	// Comparator ON...
	CMP0E1  = 0x00; 	// No Interupt...
	CMP0E0  = 0x00;
	CMP0SM1 = 0x00; 	// Detect without Sampling... 
	CMP0RFS = 0x01; 	// Differential Input on B5

   //Comparator 0 OFF
	CMP0EN  = 0x00;


}
//===========================================================================



//===========================================================================
//	Clear All 3 Bits of Port A
//===========================================================================
void PortA_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 0;		// PortA Bit0 set to Output Mode...
	PA1DIR = 0;		// PortA Bit1 set to Output Mode...
	PA2DIR = 0;		// PortA Bit2 set to Output Mode...

	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to CMOS Output...
	PA0C0  = 1;		
	PA1C1  = 1;		// PortA Bit1 set to CMOS Output...
	PA1C0  = 1;	
	PA2C1  = 1;		// PortA Bit2 set to CMOS Output...
	PA2C0  = 1;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose Output...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose Output...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose Output...
	PA2MD0  = 0;	

	//Data...
	PA0D = 0;		// A.0 Output OFF....
	PA1D = 0;		// A.1 Output OFF....
	PA2D = 0;		// A.2 Output OFF....

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================

//===========================================================================
//	Clear All 8 Bits of Port B
//===========================================================================
void PortB_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...
	PB1DIR = 0;		// PortB Bit1 set to Output Mode...
	PB2DIR = 0;		// PortB Bit2 set to Output Mode...
	PB3DIR = 0;		// PortB Bit3 set to Output Mode...
	PB4DIR = 0;		// PortB Bit4 set to Output Mode...
	PB5DIR = 0;		// PortB Bit5 set to Output Mode...
	PB6DIR = 0;		// PortB Bit6 set to Output Mode...
	PB7DIR = 0;		// PortB Bit7 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		
	PB1C1  = 1;		// PortB Bit1 set to CMOS Output...
	PB1C0  = 1;	
	PB2C1  = 1;		// PortB Bit2 set to CMOS Output...
	PB2C0  = 1;	
	PB3C1  = 1;		// PortB Bit3 set to CMOS Output...
	PB3C0  = 1;		
	PB4C1  = 1;		// PortB Bit4 set to CMOS Output...
	PB4C0  = 1;	
	PB5C1  = 1;		// PortB Bit5 set to CMOS Output...
	PB5C0  = 1;	
	PB6C1  = 1;		// PortB Bit6 set to CMOS Output...
	PB6C0  = 1;	
	PB7C1  = 1;		// PortB Bit7 set to CMOS Output...
	PB7C0  = 1;	

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to General Purpose Output...
	PB0MD0  = 0;	
	PB1MD1  = 0;	// PortB Bit1 set to General Purpose Output...
	PB1MD0  = 0;	
	PB2MD1  = 0;	// PortB Bit2 set to General Purpose Output...
	PB2MD0  = 0;	
	PB3MD1  = 0;	// PortB Bit3 set to General Purpose Output...
	PB3MD0  = 0;	
	PB4MD1  = 0;	// PortB Bit4 set to General Purpose Output...
	PB4MD0  = 0;	
	PB5MD1  = 0;	// PortB Bit5 set to General Purpose Output...
	PB5MD0  = 0;
	PB6MD1  = 0;	// PortB Bit6 set to General Purpose Output...
	PB6MD0  = 0;	
	PB7MD1  = 0;	// PortB Bit7 set to General Purpose Output...
	PB7MD0  = 0;

	//Data...
	PB0D = 0;		// B.0 Output OFF....
	PB1D = 0;		// B.1 Output OFF....
	PB2D = 0;		// B.2 Output OFF....
	PB3D = 0;		// B.3 Output OFF....
	PB4D = 0;		// B.4 Output OFF....
	PB5D = 0;		// B.5 Output OFF....
	PB6D = 0;		// B.6 Output OFF....
	PB7D = 0;		// B.7 Output OFF....

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================

//===========================================================================
//	Clear All 4 Bits of Port C
//===========================================================================
void PortC_Low(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PC0DIR = 0;		// PortC Bit0 set to Output Mode...
	PC1DIR = 0;		// PortC Bit1 set to Output Mode...
	PC2DIR = 0;		// PortC Bit2 set to Output Mode...
	PC3DIR = 0;		// PortC Bit3 set to Output Mode...


	//I/O Type...
	PC0C1  = 1;		// PortC Bit0 set to CMOS Output...
	PC0C0  = 1;		
	PC1C1  = 1;		// PortC Bit1 set to CMOS Output...
	PC1C0  = 1;	
	PC2C1  = 1;		// PortC Bit2 set to CMOS Output...
	PC2C0  = 1;	
	PC3C1  = 1;		// PortC Bit3 set to CMOS Output...
	PC3C0  = 1;		

	//Purpose...
	PC0MD1  = 0;	// PortC Bit0 set to General Purpose Output...
	PC0MD0  = 0;	
	PC1MD1  = 0;	// PortC Bit1 set to General Purpose Output...
	PC1MD0  = 0;	
	PC2MD1  = 0;	// PortC Bit2 set to General Purpose Output...
	PC2MD0  = 0;	
	PC3MD1  = 0;	// PortC Bit3 set to General Purpose Output...
	PC3MD0  = 0;	

	//Data...
	PC0D = 0;		// C.0 Output OFF....
	PC1D = 0;		// C.1 Output OFF....
	PC2D = 0;		// C.2 Output OFF....
	PC3D = 0;		// C.3 Output OFF....

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================

//===========================================================================
//	Set All 3 Bits of Port A as Digital Input Pins
//===========================================================================
void PortA_Digital_Inputs(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Set Pin Data...

	//Direction...	
	PA0DIR = 1;		// PortA Bit0 set to Input Mode...
	PA1DIR = 1;		// PortA Bit1 set to Input Mode...
	PA2DIR = 1;		// PortA Bit2 set to Input Mode...


	//I/O Type...
	PA0C1  = 1;		// PortA Bit0 set to Input with Pull-Up Resistor...
	PA0C0  = 0;		
	PA1C1  = 1;		// PortA Bit1 set to Input with Pull-Up Resistor...
	PA1C0  = 0;	
	PA2C1  = 1;		// PortA Bit2 set to Input with Pull-Up Resistor...
	PA2C0  = 0;	

	//Purpose...
	PA0MD1  = 0;	// PortA Bit0 set to General Purpose I/O...
	PA0MD0  = 0;	
	PA1MD1  = 0;	// PortA Bit1 set to General Purpose I/O...
	PA1MD0  = 0;	
	PA2MD1  = 0;	// PortA Bit2 set to General Purpose I/O...
	PA2MD0  = 0;	

	main_clrWDT(); 	// Clear WDT

}
//===========================================================================



//===========================================================================
//	PWM Output on Port B - Pin 0
//===========================================================================
void PinB0_PWM(void){

//Carl's Notes...

//Step 1: Set Pin Direction...
//Step 2: Set Pin I/O Type...
//Step 3: Set Pin Purpose...
//Step 4: Select the Clock Mode...
//Step 5: Set the Duty Cycle...
//Step 5: Start the PWM Counter...

	//Direction...	
	PB0DIR = 0;		// PortB Bit0 set to Output Mode...

	//I/O Type...
	PB0C1  = 1;		// PortB Bit0 set to CMOS Output...
	PB0C0  = 1;		

	//Purpose...
	PB0MD1  = 0;	// PortB Bit0 set to PWM Output (0,1)...
	PB0MD0  = 1;	


	//Select the Clock Mode...
	PCCS1 = 0;	//00= LS; 01=HS; 10=PLL
	PCCS0 = 1;

	//SET THE PERIOD...(Added April 4th, 2013)
	PWCP = 4250;		// Init Period to (1=255kHz; 10=46kHz; 50=10kHz; 200=2.5kH; ; 3185 = 160Hz; 3400=150Hz; 4250=120Hz; 5000=102Hz)

	//SET THE DUTY CYCLE...(Added April 15th, 2013)

	//PWCD =    10;		//10    ~  0.2 % duty cycle @ 120Hz
	//PWCD =   100;		//100   ~  2.4 % duty cycle @ 120Hz
	//PWCD =  1000;		//1000  ~ 23.5 % duty cycle @ 120Hz
	//PWCD = 4000;		//4000  ~ 94.0 % duty cycle @ 120Hz
	//PWCD = 4150;		//4150  ~ 99.0 % duty cycle @ 120Hz
	//PWCD =    20;		//20    ~  0.4 % duty cycle @ 120Hz	
	PWCD =    12;		//12    ~  0.25% duty cycle @ 160Hz

	PCRUN = 0;		// OFF to start

}
//===========================================================================