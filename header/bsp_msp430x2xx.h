#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx
 //#include  <msp430xG46x.h>  // MSP430x4xx
//////////////////////////////////////
//P1.0 - Analog input signal (channel A0) - LDR1_CH_I
//P1.1 - UART
//P1.2 - UART
//P1.3 - Analog input signal (channel A3) - LDR2_CH_I
//P1.4 - LCD Data D4
//P1.5 - LCD Data D5
//P1.6 - LCD Data D6
//P1.7 - LCD Data D7=


//P2.1:LCD control signal E

//P2.2: Ultrasonic sensor ECHO : Timer A1 CCR1

//P2.3: LCD control signal RS

//P2.4: Timer A1 CCR2 : PWM out (for servo motor) - Timer A1 CCR2

//P2.5: LCD control signal RW

//P2.6 :Ultrasonic sensor input signal Trig : timer A0 CCR1

//P2.7: reserved for SEL



//////////////////////////////////////

#define   debounceVal      250
#define   LEDs_SHOW_RATE   6800//0x1FE8 ,each delay is 10 cycle's so delay 6551 we take 7000 // 62_5ms
#define   PWM_delay        1
#define   Half_sec         52428  // from calculate cycle for half sec


// PushButtons abstraction
#define PBsArrPort	       P2IN
#define PBsArrIntPend	   P2IFG
#define PBsArrIntEn	       P2IE
#define PBsArrIntEdgeSel   P2IES
#define PBsArrPortSel      P2SEL
#define PBsArrPortDir      P2DIR
#define PB0                0x01
#define PB1                0x80


// Generator abstraction
#define GenPort            P2IN
#define GenPortSel         P2SEL
#define GenPortSel2        P2SEL2
#define GenPortDir         P2DIR

// Buzzer abstraction


// PWM OUT POT
#define PWMArrPort            P2OUT
#define PWMArrPortDir         P2DIR
#define PWMArrPortSel         P2SEL
// UART
#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

// LCD abstraction

//P2.1:LCD control signal E|P2.3: LCD control signal RS|P2.5: LCD control signal RW
// Control port for LCD P2.
#define LCDC_PORT           P2OUT
#define LCDC_DIR            P2DIR
#define LCDC_SEL            P2SEL
#define LCDC_EN_PIN         0x02
#define LCDC_RS_PIN         0x08
#define LCDC_RW_PIN         0x20
#define LCDC_PINS           0x2A
// Data port for LCD
#define LCDD_PORT           P1OUT
#define LCDD_DIR            P1DIR
#define LCDD_SEL            P1SEL
#define LCDD_D7             0x80
#define LCDD_D6             0x40
#define LCDD_D5             0x20
#define LCDD_D4             0x10
#define LCDD_PINS           0xF0

#define LCD_DATA_OFFSET     0x04
#define OUTPUT_PIN          1
#define INPUT_PIN           0
#define FOURBIT_MODE        0x0
#define EIGHTBIT_MODE       0x1
#define LCD_MODE        F   OURBIT_MODE
////////////   LCD Macro function //////////////////
#define LCD_WAIT            delay(5);
#define OUTPUT_DATA         (0x0F << LCD_DATA_OFFSET)
#define LCD_EN(a)           (!a ? (LCDC_PORT &= ~LCDC_EN_PIN) : (LCDC_PORT |= LCDC_EN_PIN)) // LCD Enable signal control
#define LCD_RS(a)           (!a ? (LCDC_PORT &= ~LCDC_RS_PIN) : (LCDC_PORT |= LCDC_RS_PIN))
#define LCD_RW(a)           (!a ? (LCDC_PORT&=~LCDC_RW_PIN) : (LCDC_PORT|=LCDC_RW_PIN)) // P2.5 is lcd RW pin
#define lcd_cursor_right()  lcd_cmd(0x14)

//---------------  TIMER1  --------------------------
#define Timer_1_CTL         TA1CTL
#define Timer_1_CLR         TACLR
#define Timer_1_CCTL0       TA1CCTL0
#define Timer_1_SSEL2       TASSEL_2
#define Timer_1_CCR0        TA1CCR0
#define Timer_1_CCR1        TA1CCR1
#define Timer_1_CCR2        TA1CCR2
#define Timer_1_CTL         TA1CTL
#define Timer_1_CCTL1       TA1CCTL1
#define Timer_1_CCTL2       TA1CCTL2
#define Timer_1_IFG         TAIFG
#define Timer_1_CLR         TACLR

#define Timer_1_IE          TAIE
//---------------  TIMER0  --------------------------

#define Timer_0_CCR0         TA0CCR0
#define Timer_0_CCR1         TA0CCR1
#define Timer_0_CTL          TA0CTL
#define Timer_0_CLR          TACLR
#define Timer_0_SSEL2        TASSEL_2 //SMCLK 2^20
#define Timer_0_CCTL0        TA0CCTL0
#define Timer_0_CCTL1        TA0CCTL1
#define Timer_0_IE          TAIE
////////////////////////////////////////////////


extern void TIMERA1config();
extern void TIMERA0config();
extern void TIMER_USconfig();


///////////// ADC ////////////
extern void ADC_init(void);

////////////// FLASH ///////////////////
extern void FLASHConfig();



//////////////////  Object detector state ///////////////

// Servo Motor abstraction
#define ServoMotorPortOut        P2OUT
#define ServoMotorPortDir        P2DIR
#define ServoMotorPortSel        P2SEL
#define ServoMotorPortSel2       P2SEL2
#define ServoMotorPwmLeg         0x10       //p2.4

///////////////////////////////////////////////////////
// Ultrasonic sensor abstraction


#define sensor_Trig_OUT         P2OUT
#define sensor_Trig_DIR         P2DIR
#define sensor_Trig_SEL         P2SEL
#define sensor_Trig_SEL2        P2SEL2
#define sensor_Trig_Leg         0x40       //p2.6
#define sensor_Trig_Leg2         0x80       //p2.7

/////////////  capture and compare ////////////////

#define capture_DIR         P2DIR
#define capture_SEL         P2SEL
#define capture_SEL2        P2SEL2
#define capture_Leg         0x04

////////////////  LDR /////////////////////////////

#define LDRPortsSEL         P1SEL
#define LDRPortsDIR         P1DIR
#define LDRPortOUT          P1OUT
#define LDR1_CH_I           0x01
#define LDR2_CH_I           0x08


////////////////////////////////////////////////////


/////////////  ADC abstraction ////////////////

#define ADC_CTL0            ADC10CTL0
#define ADC_CTL1            ADC10CTL1
#define ADC_ON              ADC10ON
#define ADC_IE              ADC10IE
#define ADC_SHT3            ADC10SHT_3
#define ADC_SSEL_3          ADC10SSEL_3
#define ADC_CTL1            ADC10CTL1
#define ADC_SSEL_3          ADC10SSEL_3
#define ADC_AE0             ADC10AE
#define ADC_SC              ADC10SC
#define ADC_IE              ADC10IE
#define ADC_AE0             ADC10AE0
#define ADC_SEMP            ADC10MEM





///////////////////////////////////////////////
/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~LCDC_EN_PIN) : (P2OUT|=LCDC_EN_PIN)) // P2.1 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~LCDC_EN_PIN) : (P2DIR|=LCDC_EN_PIN)) // P2.1 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~LCDC_RS_PIN) : (P2OUT|=LCDC_RS_PIN)) // P2.3 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~LCDC_RS_PIN) : (P2DIR|=LCDC_RS_PIN)) // P2.3 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~LCDC_RW_PIN) : (P2OUT|=LCDC_RW_PIN)) // P2.5 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~LCDC_RW_PIN) : (P2DIR|=LCDC_RW_PIN)) // P2.5 pin direction

#define lcd_clear()             lcd_cmd(0x01)
#define lcd_home()              lcd_cmd(0x02)
#define lcd_cursor_right()      lcd_cmd(0x14)
#define lcd_cursor_left()       lcd_cmd(0x10)
#define lcd_new_line()            lcd_cmd(0xC0)
#define cursor_off()              lcd_cmd(0x0C)
#define cursor_on()               lcd_cmd(0x0F)
#define lcd_goto(x)     lcd_cmd(0x80+(x))

extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
extern void GPIOconfig(void);
extern void UART_init(void);
extern int d;     // delay value ms
extern int d_val;      // delay value after calculating


#endif



