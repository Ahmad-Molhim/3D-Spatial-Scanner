#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0b00001111;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0b00001111;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0b00001111;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0b00001111;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10; //activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; //allow time for clock to stabilize
	
	GPIO_PORTL_DIR_R = 0b00000000; // Make PL0 input, from the button press
	GPIO_PORTL_DEN_R = 0b00000001; // Enable PL0

	return;
}

void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; //allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R = 0b00010011; // Make PF0, PF4 and PF1 output
	GPIO_PORTF_DEN_R = 0b00010011; // Enable PF4
	
return;
}

void PortN_Init(void){
    //Use PortN onboard LEDs
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                        // Activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};    // Allow time for clock to stabilize
    GPIO_PORTN_DIR_R |= 0x07;                                                // Make PN0 and PN1 out (PN0/PN1 built-in LEDs)
    GPIO_PORTN_AFSEL_R &= ~0x07;                                             // Disable alt funct on PN[0:1]
  GPIO_PORTN_DEN_R |= 0x07;                                                // Enable digital I/O on PN[0:1]
                                                                                                    
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x07;                                             // Disable analog functionality on PN[0:1]
    //FlashLED1(1);                                                                                // Flash LED D1 (Hello World)
    return;
}



//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashLED4(1);
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void spin_CCW(){																			// Complete function spin to implement the Full-step Stepping Method
	uint32_t delay = 1;															// Does your motor spin clockwise or counter-clockwise?
	
	for(int i=0; i<512; i++){												// What should the upper-bound of i be for one complete rotation of the motor shaft?
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(delay);
		}
	}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t modelID;
	uint8_t modType;
	uint16_t typeAndId;
	

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortL_Init();
	PortF_Init();
	uint32_t delay = 1;

	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// 1 Wait for device booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

    status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	
	// Get the Distance Measures 50 times
	for(int i = 0; i < 10; i++) {
		
		// 5 wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//7 read the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);					
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
		status = VL53L1_RdByte(dev, 0x010F, &modelID); //for model ID (0xEA)
    status = VL53L1_RdByte(dev, 0x0110, &modType); //for module type (0xCC)
		status = VL53L1_RdWord(dev, 0x010F, &typeAndId); //for both model ID and type
		
 

	  status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
		
		while(1){
		if ((GPIO_PORTL_DATA_R & 0b00000001) == 0x01){
			for(int i = 1; i <= 1536; i++){
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(delay); 
				if (i % 64 == 0) {
					//Flash LED D3 
				GPIO_PORTF_DATA_R=0b00010000;
			  SysTick_Wait10ms(delay);
				GPIO_PORTF_DATA_R=0b00000000;
					//Get distance measurement 
				status = VL53L1X_GetDistance(dev, &Distance);				
				sprintf(printf_buffer,"%u\r\n", Distance);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(50);					
			}
		}

	 	GPIO_PORTF_DATA_R=0b00000001;
		SysTick_Wait10ms(delay);
		GPIO_PORTF_DATA_R=0b00000000;
		
	}			
}		
		
		// print the resulted readings to UART
		
	  //
  }
  
	VL53L1X_StopRanging(dev);
  while(1) {}
		

}


