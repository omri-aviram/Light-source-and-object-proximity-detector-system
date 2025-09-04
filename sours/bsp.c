#include  "../header/bsp_msp430x2xx.h"    // private library - BSP layer


int d = 50;     // delay value ms
int d_val;      // delay value after calculating


//-----------------------------------------------------------------------------  
//           GPIO congiguration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
  

//////////////// LCD
  // P2.1:LCD control signal E|P2.3:LCD control signal RS|P2.5:LCD control signal RW
  LCDC_SEL &= ~LCDC_PINS;
  LCDC_DIR |= LCDC_PINS;
  LCDC_PORT &= ~LCDC_PINS;
//////////////// Data port LCD
  LCDD_SEL &= ~LCDD_PINS;
  LCDD_DIR |= LCDD_PINS;
  LCDD_PORT &= ~LCDD_PINS;

////////////Servo motor setup P2.4
     ServoMotorPortSel        |=   ServoMotorPwmLeg;   // Timer Mode
     ServoMotorPortDir        |=   ServoMotorPwmLeg;   //Output mode
     ServoMotorPortSel2       &=  ~ServoMotorPwmLeg;
     ServoMotorPortOut        &=  ~ServoMotorPwmLeg;

////////////Ultrasonic sensor setup P2.6
     sensor_Trig_DIR        |=   sensor_Trig_Leg;   //Output mode
     sensor_Trig_SEL        |=   sensor_Trig_Leg;   // Timer Mode
     sensor_Trig_SEL        &=   ~sensor_Trig_Leg2;   // Timer Mode P2.7 zero
     sensor_Trig_SEL2       &=  ~(sensor_Trig_Leg|sensor_Trig_Leg2);
//////////////////  capture and compar
     capture_SEL           |=  BIT2;
     capture_DIR           &= ~BIT2;
     capture_SEL2          &= ~BIT2;

/////////////////// LDR
     // P1.0
     LDRPortsSEL        &=  ~LDR1_CH_I;  // IO
     LDRPortsDIR        &=  ~LDR1_CH_I;  // INPUT
     //P1.3
     LDRPortsSEL        &=  ~LDR2_CH_I;
     LDRPortsDIR        &=  ~LDR2_CH_I;

/////////////// PushButtons setup
     PBsArrPortSel &= ~(PB0|PB1);       // SEL 0 is I/O P2.0,P2.7
     PBsArrPortDir &= ~(PB0|PB1);           // DER 0 is I/O P2.0,P2.7
     PBsArrIntEdgeSel |= (PB0|PB1);          // pull-up mode
     //PBsArrIntEdgeSel &= ~0x0C;         // pull-down mode no need
     PBsArrIntEn |= (PB0|PB1);              // enable interrupt
     PBsArrIntPend &= ~(PB0|PB1);            // clear pending interrupts









  _BIS_SR(GIE);                     // enable interrupts globally
}                             
//------------------------------------------------------------------------------------- 
//            Timers congiguration 
//-------------------------------------------------------------------------------------

void TIMERA1config(){
    Timer_1_CTL =  MC_1 +Timer_1_SSEL2 ;    // Continuous mode + SMCLK
}

//////  we exit Slipping mode after 16 interrupts 2^4,2^1,2^11
void TIMERA0config(){
    d_val = 4*d*10;                     // default is 5 sec 4*50*10 ~ 2^11 for d=50
    Timer_0_CCR0   = (unsigned int)d_val;
    Timer_0_CTL    = MC_3 + Timer_0_SSEL2 + ID_3 + Timer_0_CLR;   //UpDown + SMCLK + Div by 8 +
}                                                                   // + Clear Timer

void TIMER_USconfig(){

    Timer_0_CTL =  MC_1 +Timer_0_SSEL2 ;// Continuous mode + SMCLK

    Timer_1_CTL    = MC_2 + Timer_1_SSEL2;              // Continous, smclk

    Timer_1_CCTL1  = CAP | CCIE | CCIS_1 | CM_3 | SCS;  //  Capture mode + Enable interrupts +
                                                        //+ Capture/compare input (p2.2) +
                                                        //+ Rising and Falling edge + Synchronize

}

//-------------------------------------------------------------------------------------
//            ADC configuration
//-------------------------------------------------------------------------------------

//P1.0 - Analog input signal (channel A0) - LDR1
void ADC_init(void){

    Timer_1_CTL =  MC_1 +Timer_1_SSEL2 ; // smclk , cintnous

    ADC_CTL0 = ADC_ON + ADC_IE+ADC_SHT3 +SREF_0;

    ADC_CTL1 = INCH_0 + ADC_SSEL_3 ;

    ADC_AE0 |= LDR1_CH_I; // P1.0

    ADC_AE0 &= LDR2_CH_I; // P1.3


}
//-------------------------------------------------------------------------------------
//            Flash congiguration
//------------------------------------------------------------------------------------
void FLASHConfig(){
        if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
             {
               while(1);                               // do not load, trap CPU!!
             }
             DCOCTL = 0;                               // Select lowest DCOx and MODx settings
             BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
             DCOCTL = CALDCO_1MHZ;
             FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing
    }
//------------------------------------------------------------------------------------- 
//            UART_init congiguration
//-------------------------------------------------------------------------------------


void UART_init(void){
    if (CALBC1_1MHZ==0xFF)                  // If calibration constant erased
      {
        while(1);                               // do not load, trap CPU!!
      }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    //P2DIR |= 0xFB;                             // All P2.x outputs
    //P2OUT = 0;                                // All P2.x reset
    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    //P1DIR |= RXLED + TXLED;
    //P1OUT &= 0x00;

    UCA0CTL1 |= UCSSEL_2;                     // CLK = SMCLK
    UCA0BR0 = 104;                           //
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS0;               //
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX
}
           
             

 
             
             
            
  

