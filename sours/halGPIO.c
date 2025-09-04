#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
#include <string.h>

//////////////////////// Variable Declaration /////////////////////////////
enum objcet_func detec_state;
enum LDRstate light_detector_state;
enum FSMstate state;
enum SYSmode lpm_mode;
enum TelemeterState Telemeter_state;
enum FileMode FileMode_state , FileMode_state_temp;
enum Ftype F_type;
enum LightSourcesObjects SourcesObjects_state; //global variable

////////////////// timer A0 ///////////
int timer_counter;
/////////// echo variable ////////////
int echo_index=0;
int echo_samp [2];
int t_diff,temp;
int measure_result[4] ={'0','0','0','\n'}; //might not be in use
unsigned char diff_bytes[2];
unsigned int diff_tx_i = 0;
unsigned char diff_flag = 0;
int initial_engal;
int dig ;

//////////// LDR Variables ////////
char ScriptName [16]= { {0} };
int S_index = 0 ;
int LDR_samp1=0;
int LDR_samp2=0;
int avg_LDRsample=0;
//int LDR_sample; // can be change
int LDR_sample;
unsigned char LDR_bytes[2];

//unsigned char LDR_bytes;
unsigned int LDR_tx_i = 0;
unsigned int LDR_flag = 0;
unsigned int Tel_tx_i = 0;

//////////// Telemeter Variables ////////
int Tel_angle_idx = 0;
int Tel_flag = 0;
char Tel_angle_rx[4] = {'0','0','0','\n'};
int Tel_scanFlag = 0;
unsigned char Tel_bytes[2] ;
int tele_angle_int ;

//////////////////  FileMode  //////////////////////
int get_Name_flag=0;
int SName_flag = 0;
int file_flag = 0;// here i change to 0
int F_length ,F_index ;
char* FLASH_ptr;
volatile unsigned int  F_sum = 0;
volatile unsigned char F_chk  = 0;
int first_FileUp = 0 ;
volatile unsigned char pb_event = 0;

////////////////  NEW  ////////////////////////////////

static volatile unsigned int  rx_addr  = 0;
static volatile int           rx_slot  = 0;

////////////////// NEW //////////////////////////////
FSHeader fs = {  //change here !!!!!!!!!!!!!!!!!!!!!!!
    .next_free = FILES_REGION_START,
    .num_files = 0,
    .crc       = 0,
    .entries   = { {0} }
};



///////////////////////// calibration //////////////////////
volatile unsigned char cal_tx_flag = 0;
static unsigned char cal_tx_phase = 0;
static unsigned char cal_tx_sub = 0;    // sending LSB or MSB
static unsigned int  cal_tx_idx = 0;    // index of sample
static unsigned char cal_tx_sensor = 1; // sensor number 1 or 2

//////////////////////// Light Sources Objects ///////////////////////

unsigned int Sourc_Obj_flag = 0 ;


/////////////////////////////////////////////////////////////



static unsigned int current_seg_base = 0xFFFF;

const char* fs_get_entry_name_ptr(unsigned int n){
    const FSHeader* fs_trmpPtr = fs_map();
    if(n >= 10 || n >= fs_trmpPtr->num_files) return NULL;
    return (const char*)(unsigned int)(fs_trmpPtr->entries[n].start);
}

char* fs_get_entry_name(unsigned int n){
    const FSHeader* fs_tempName = fs_map();
    if(n >= 10 || n >= fs_tempName->num_files) return NULL;
    const char* p = (const char*)(unsigned int)(fs_tempName->entries[n].start);
    static char s[17];
    int i = 0;
    for(; i < 16 && p[i] != 0; i++) s[i] = p[i];
    s[i] = 0;
    return s;
}

////////////////// NEW2////////////////////////////////
static inline const FSHeader* fs_map(void){ // return ptr to FSHeader
    return (const FSHeader*)(unsigned int)FS_INFO_ADDR;/* segmant C 0x1040 */
}
//////////////////////////////////////

unsigned int cal1[CAL_POINTS];
unsigned int cal2[CAL_POINTS];

/////////////////////////////              FUNCTION           ///////////////////////////////////////////


//--------------------------------------------------------------------
//             System Configuration
//--------------------------------------------------------------------
void sysConfig(void){
    GPIOconfig();
    lcd_init();
    UART_init();
    FLASHConfig();
    first_FileUp = 0;
    //load_fs();
    //fs_load();
    cal_load();
    //cal_build_lut();
}







//static inline void fs_load(FSHeader* dst){
//    const void* src = (const void*)(unsigned int)FILES_REGION_START;
//    memcpy(dst, src, sizeof(FSHeader));
//}


////////////////////////// get index by Name of script ///////////////////////
int fs_find_index_by_name(const char* arr_name){
    const FSHeader* h = fs_map();
    if(!h) return -1;
    unsigned int i;
    for(i = 0; i < h->num_files && i < 10; i++){
        const FileEntry* e = &h->entries[i];
        if(!e->in_use) continue;
        const char* p = (const char*)(unsigned int)e->start;
        char name[17];
        unsigned int j = 0;
        for(; j < 16; j++){
                   char a = arr_name[j];
                   char b = p[j];
                   if(a != b) break;// not same Name
                   //if(a == 0) break;// end of Name
               }
        //for(; j < 16 && p[j] != 0; j++) name[j] = p[j];
        //name[j] = 0;
        //if(strcmp(name, arr_name) == 0) return (int)i;
        if(j == 16 ) return (int)i;
    }
    return -1;
}
////////////////////////////////////////////////////
void load_fs(){
     fs = *fs_map();
}



/////////////////////// File mode finction ////////////////////////////////


void fs_load(void){
    const unsigned char *src = (const unsigned char*)FS_INFO_ADDR; // initial adders of fill manager in flash
    FSHeader cand;
    memcpy(&cand, src, sizeof(FSHeader));// copy memory from flash to cand (RAM)

    unsigned char old_crc = cand.crc; // check sum val
    cand.crc = 0;
    unsigned char calc = fs_crc8_calc((unsigned char*)&cand, sizeof(FSHeader));


    if (old_crc == calc &&
        cand.num_files <= 10 &&
        cand.next_free >= FILES_REGION_START &&
        cand.next_free <= (FILES_REGION_END + 1)) {
        fs = cand;
    }
    else {
        // default FS
        memset(&fs, 0, sizeof(fs));
        fs.next_free = FILES_REGION_START;
        fs.crc = 0;
        fs_store();  // initialize persistent FS
    }
}


//------------------------------------------------------------------------
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ return the last 8 bit of sum
static unsigned char fs_crc8_calc(const unsigned char *p, unsigned int len){
    int i;
    unsigned int s = 0;
    for (i = 0; i < len; i++) s = (s + p[i]) & 0xFF;
    return (unsigned char)s;
}


//------------------------  Store fs ------------------------------
void fs_store(void){
    int i ;
    unsigned char buf[sizeof(FSHeader)] ;// temp buffer
    FSHeader *tmp = (FSHeader*)buf;
    FLASHConfig();
    *tmp = fs;
    tmp->crc = 0;
    tmp->crc = fs_crc8_calc(buf, sizeof(FSHeader)); // get chick sum to temp

    // erase+write Info segment at FS_INFO_ADDR
    flash_write_init((int)FS_INFO_ADDR);               // ERASE + WRT on Info Flash segment
    for ( i = 0; i < sizeof(FSHeader); i++) Char2Flah(buf[i]);
    dis_FLASHWrite();
}









//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init()
{
    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCDD_PORT |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);
    DelayMs(15);
    LCDD_PORT &= ~OUTPUT_DATA;
    LCDD_PORT |= init_value;
    lcd_strobe();
    DelayMs(5);

    LCDD_PORT &= ~OUTPUT_DATA;
    LCDD_PORT |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCDD_PORT &= ~OUTPUT_DATA;
    LCDD_PORT |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT;
        LCDD_PORT &= ~OUTPUT_DATA;
        LCDD_PORT |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF);  // Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1);  // Display Clear
    lcd_cmd(0x6);  // Entry Mode
    lcd_cmd(0x80); // Initialize DDRAM address to zero
    cursor_off(); // cursor off
}

//******************************************************************
// open new segment exactly on boundary (kept as-is)
//******************************************************************
static inline void flash_advance_segment_if_needed(void){
    unsigned int cur = (unsigned int)FLASH_ptr;
    if ((cur & (FILE_SEG_SIZE - 1)) == 0){
        dis_FLASHWrite();
        flash_write_init((int)cur);
    }
}

//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char comm){

    if (LCD_MODE == FOURBIT_MODE){
        LCD_RS(0);
        DelayMs(5);
        LCDD_PORT &= ~OUTPUT_DATA;
        LCDD_PORT |= ((comm >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCDD_PORT &= ~OUTPUT_DATA;
        LCDD_PORT |= (comm & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else {
        LCDD_PORT = comm;
        lcd_strobe();
    }
}

//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm(" nop");
  asm(" nop");
  LCD_EN(0);
}

//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_RS(1);
    DelayMs(5);
    LCDD_PORT &= ~OUTPUT_DATA;
    if (LCD_MODE == FOURBIT_MODE)
    {
        LCDD_PORT &= ~OUTPUT_DATA;
        LCDD_PORT |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCDD_PORT &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
        LCDD_PORT &= ~OUTPUT_DATA;
        LCDD_PORT |= (c & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCDD_PORT = c;
        lcd_strobe();
    }

    LCD_RS(0);
}

//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char *s){
  while(*s)
      lcd_data(*s++);
}

//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop");
}

//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) {
        DelayUs(1000);
    }
}

//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){
    volatile unsigned int i;
    for(i=t; i>0; i--);
}

//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){

    if (LPM_level == 0x00)
        _BIS_SR(LPM0_bits);
    else if(LPM_level == 0x01)
        _BIS_SR(LPM1_bits);
    else if(LPM_level == 0x02)
        _BIS_SR(LPM2_bits);
    else if(LPM_level == 0x03)
        _BIS_SR(LPM3_bits);
    else if(LPM_level == 0x04)
        _BIS_SR(LPM4_bits);
}

//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}

//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}

///////////////////////////  OUR FUNCTION //////////////////////////////////

//---------------------------------------------------------------------
//            Servo PWM
//---------------------------------------------------------------------
void PWMGEN_Servo(int feq){
    Timer_1_CCR0 = (int)20000; // PWM period 20000 change to 30000, 30_8 26000!!!!!!!!!!!!!!!!!!!!!
    Timer_1_CCR2 = feq;        // duty
    Timer_1_CCTL2 = OUTMOD_7;  // reset/set
    Timer_1_CTL =  MC_1 + Timer_1_SSEL2; // up to CCR0 + SMCLK
}

void PWM_OFFServo(){
    Timer_1_CCTL2 &= ~OUTMOD_7;
}

//---------------------------------------------------------------------
//            PWM TRIGGER OUT
//---------------------------------------------------------------------
void PWMTriggerGEN(){
    Timer_0_CCR0  = 65535;
    Timer_0_CCR1  = 10;
    Timer_0_CCTL1 |= OUTMOD_7;
    Timer_0_CTL   = MC_1 + Timer_0_SSEL2;
}


void PWMTriggerDisable(){

    Timer_0_CTL   &= ~Timer_0_IE;
    Timer_0_CCTL1 &= ~CCIE;
    Timer_0_CCTL1  = OUTMOD_5;

}

//-------------------------------------------------------------
//             enable/disable Timer's
//-------------------------------------------------------------
void enable_timerA0(){
    Timer_0_CCTL0 |=  CCIE;
}

void disable_timerA0(){
    Timer_0_CCTL0 &= ~CCIE;
}

void disable_timerA1(){
    Timer_1_CTL   &= ~Timer_1_IE;
    Timer_1_CCTL1 &= ~CCIE;
    Timer_1_CCTL2 &= ~CCIE;
    Timer_1_CCTL2  = OUTMOD_5;
   // Timer_1_CCTL2 &= ~OUTMOD_7; // pwm dont stop
}


void clear_timerA0(){

    Timer_0_CTL |= Timer_0_CLR; // clecr timer 0 CTL


}



//---------------------- UART function --------------------------
void send_samp(){
    diff_flag = 0;
    diff_tx_i = 0;
    LDR_tx_i = 0;                 // <-- keep this line
    LDR_flag = 1;
    IFG2 &= ~UCA0TXIFG;
    //IE2 |= UCA0TXIE;

    UCA0TXBUF = (unsigned char)(dig & 0xFF);
    //if(Sourc_Obj_flag == 0)UCA0TXBUF = (unsigned char)(dig & 0xFF);   // first byte = angle
    //else if(Sourc_Obj_flag == 1)UCA0TXBUF = (unsigned char)LDR_bytes[LDR_tx_i++];
    IE2 |= UCA0TXIE;                  // TX ISR will send the 2 LDR bytes
}


/*void send_samp(){
    diff_flag = 0;
    diff_tx_i = 0;
    LDR_bytes = (unsigned char)(LDR_sample & 0xFF);
    //LDR_bytes[1] = (unsigned char)((LDR_sample >> 8) & 0xFF);
    //LDR_tx_i = 0;
    LDR_flag = 1;
    UCA0TXBUF = (unsigned char)dig;
    IE2 |= UCA0TXIE;
}*/









//----------------------ADC functions-----------------------------
void enable_samp(){
    ADC_CTL0 |=  ADC_ON + ENC + ADC_SC;
    ADC_CTL0 |= ADC_IE;
}

void disable_ADC(){
   ADC_CTL0 &= ~ENC;
   ADC_CTL0 &= ~ADC_ON;
   ADC_CTL0 &= ~ADC_SC;
   ADC_CTL0 &= ~ADC10IE;
}

void init_LDR1(){
    ADC_CTL1 = INCH_0 + ADC_SSEL_3;
}

void init_LDR2(){
    ADC_CTL1 = INCH_3 + ADC_SSEL_3;
}
/*
unsigned int get_sample(){
    return (unsigned int)ADC_SEMP;
}
*/
/*unsigned char get_sample(){
    return (unsigned int)(ADC_SEMP >> 2);
}*/
unsigned char get_sample(){
    return (unsigned char)(ADC_SEMP >> 2);
}
//----------------------    Flash Functions -----------------------------
//-------------------------------------------------------------
//                      FLASH DISABLE WRITE
//-------------------------------------------------------------
void dis_FLASHWrite(){
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
}

//-------------------------------------------------------------
//               initialize Flash Write (with ERASE)
//-------------------------------------------------------------
void flash_write_init(int addr){
    FLASH_ptr = (char *) addr;
    FCTL1 = FWKEY + ERASE;
    FCTL3 = FWKEY;
    *FLASH_ptr = 0;                 // dummy write to erase segment
    FCTL1 = FWKEY + WRT;
}

//-------------------------------------------------------------
//               initialize Flash Write (NO ERASE)  [NEW]
//-------------------------------------------------------------
static inline void flash_wrt_no_erase(int addr){
    FLASH_ptr = (char *)addr;
    FCTL1 = FWKEY;        // no ERASE
    FCTL3 = FWKEY;        // unlock
    FCTL1 = FWKEY + WRT;  // enable write
}

//-------------------------------------------------------------
//              Write one char to Flash
//-------------------------------------------------------------
void Char2Flah(char CH ){
    *FLASH_ptr++ = (char)CH;
}
//-----------------------------------------------------------------------------
//----------------------    CALIBRATION Functions -----------------------------
//-----------------------------------------------------------------------------

//////////////////////   LDR SAMPLE ARRAY FROM FLAS ///////////////////////////////////////
void cal_load(void){
    unsigned char buf[64];
    const unsigned char *src1 = (const unsigned char*)CAL1_ADDR;// POINTER TWO SAMPLE ARRAY IN FLASH
    const unsigned char *src2 = (const unsigned char*)CAL2_ADDR;// POINTER TWO SAMPLE ARRAY IN FLASH
    unsigned char crc, old;

    memcpy(buf, src1, 64); // COPY SAMPLE POINT ARRAY FROM FLASH TWO BUFER

    old = buf[2]; // CHECK SUM FROM FLASH MEM

    buf[2] = 0; // CLEAR CHECK SUMS

    crc = cal_crc8(buf, 64); // CALC CHECK SUM

    if (buf[0] == 0xC1 && buf[1] == CAL_POINTS && crc == old){// FLASH MEMORY IS VALID
        unsigned int i;
        const unsigned char *p = &buf[4]; // PTR TWO START OF SAMPLE ARRAY
        for (i=0;i<CAL_POINTS;i++)
        {
            cal1[i] = (unsigned int)p[0] | ((unsigned int)p[1] << 8); // GET VALUE FROM FLASH TO cal1
            p += 2;

        }

    }

    else
    { // VALUE NOT VALID
        memset(cal1, 0, sizeof(cal1));

    }
    memcpy(buf, src2, 64);
    old = buf[2];
    buf[2] = 0;
    crc = cal_crc8(buf, 64);
    if (buf[0] == 0xC2 && buf[1] == CAL_POINTS && crc == old) //data valid
    {
        unsigned int i;
        const unsigned char *p = &buf[4];//where the samples begin
        for (i=0;i<CAL_POINTS;i++){
            cal2[i] = (unsigned int)p[0] | ((unsigned int)p[1] << 8);
            p += 2;
        }

    }

    else
    {
        memset(cal2, 0, sizeof(cal2));
    }

}

////////////////////////////////////////////// CALIBRATION Interpolation arrays calculate  //////////////////////////////////////////////


//////////////  CHECK SUM FOR CALIBRATION ////////////////////////
unsigned char cal_crc8(const unsigned char *p, unsigned int len){
    unsigned int s = 0;
    unsigned int i;
    for (i = 0; i < len; i++) s = (s + p[i]) & 0xFF;
    return (unsigned char)s;
}


////////////////////    Interrupt Service Routines            ////////////////////////////////////

//*********************************************************************
//            Port2 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void PBs_handler(void){

    delay(LEDs_SHOW_RATE);

    if(PBsArrIntPend & PB0){
        PBsArrIntPend &= ~PB0;
        pb_event = 1;
        //__bic_SR_register_on_exit(LPM0_bits);
    }
    if(PBsArrIntPend & PB1){
        PBsArrIntPend &= ~PB1;
        pb_event = 2;
       // __bic_SR_register_on_exit(LPM0_bits);
    }



    switch(lpm_mode){
        case mode0: LPM0_EXIT; break;
        case mode1: LPM1_EXIT; break;
        case mode2: LPM2_EXIT; break;
        case mode3: LPM3_EXIT; break;
        case mode4: LPM4_EXIT; break;
    }
}

//*********************************************************************
//            Timer1 Interrupt Service Routine
//*********************************************************************
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_1_ISR(void){

    echo_samp[echo_index] = (int)Timer_1_CCR1;
    echo_index += 1;
    Timer_1_CCTL1 &= ~CCIFG;
    Timer_1_CTL   &= ~Timer_1_IFG;

    if(echo_index >= 2 ){
        t_diff = echo_samp[1] - echo_samp[0];
        diff_bytes[0] = (unsigned char)(t_diff & 0xFF);
        diff_bytes[1] = (unsigned char)((t_diff >> 8) & 0xFF);
        diff_tx_i = 0;
        diff_flag = 1;
        if(Tel_scanFlag==1)dig = tele_angle_int ;
          UCA0TXBUF = (unsigned char)dig;
          IE2 |= UCA0TXIE;
          echo_index = 0;
          LPM0_EXIT;
        //}
    }
}

//*********************************************************************
//            TimerA0 Interrupt Service Routine
//*********************************************************************
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void) {
    if (timer_counter==16){
    timer_counter = 0;
    __bic_SR_register_on_exit(LPM0_bits); //out from sleep (+GIE?)
    }
    timer_counter++;
}

//*********************************************************************
//            ADC10 Vector Interrupt Service Routine
//*********************************************************************
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    __bic_SR_register_on_exit(LPM0_bits);
}

//*******************************************************************
//            UART TX Interrupt Service Routine
//*******************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (diff_flag== 1){// &&Tel_scanFlag == 0) {
        //LDR_tx_i = 0;
        LDR_flag = 0;
        UCA0TXBUF = diff_bytes[diff_tx_i++];
        if (diff_tx_i >= 2) {
            diff_flag = 0;
            IE2 &= ~UCA0TXIE;
            //if ((!LDR_flag) || (!Tel_scanFlag))  IE2 &= ~UCA0TXIE;
        }
    }
    else if (LDR_flag) {
        // send two bytes: LDR1 then LDR2
        UCA0TXBUF = LDR_bytes[LDR_tx_i++];

        if (LDR_tx_i >= 2){// ||(Sourc_Obj_flag == 1 && LDR_tx_i >=1 ) ) {          // done sending both bytes
            LDR_flag = 0;
            Sourc_Obj_flag=0;
            IE2 &= ~UCA0TXIE;         // stop TX interrupts for this sample
            //LPM0_EXIT;
        }
    }


    else if (cal_tx_flag){
              if (cal_tx_phase == 0){
                  UCA0TXBUF = (unsigned char)CAL_POINTS; // sending NUM of sample point to PC
                  cal_tx_phase = 1; // sanding samples
                  cal_tx_sensor = 1; // sensor one
                  cal_tx_idx = 0;
                  cal_tx_sub = 0;
              }
      else
              {
                  unsigned int v = (cal_tx_sensor == 1) ? cal1[cal_tx_idx] : cal2[cal_tx_idx]; // gey valu fronm sensor 1 or 2
                  if (cal_tx_sub == 0){
                      UCA0TXBUF = (unsigned char)(v & 0xFF); // send LSB
                      cal_tx_sub = 1;
                  }
                  else
                  {
                      UCA0TXBUF = (unsigned char)((v >> 8) & 0xFF);// send MSB
                      cal_tx_sub = 0;
                      cal_tx_idx++;
                  }
                  if (cal_tx_idx >= CAL_POINTS){ // Finish sending sample's
                      if (cal_tx_sensor == 1){
                          cal_tx_sensor = 2;// sending sensor 2 samples now
                          cal_tx_idx = 0;
                          cal_tx_sub = 0;
                      }
                      else
                      {
                          cal_tx_flag = 0; // finish sending sensor 2 samples
                          IE2 &= ~UCA0TXIE; // disable interrupts
                          LPM0_EXIT;
                      }
                  }
              }
          }









}

//*******************************************************************
//            UART RX Interrupt Service Routine
//*******************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    //////// Telemeter states //////////////////////////
    if (Tel_flag == 1){
        Tel_angle_rx[Tel_angle_idx++] = UCA0RXBUF;
        if (Tel_angle_idx==3){
            Tel_flag = 0;
            LPM0_EXIT;
        }
    }
    ///////////  File state FSM  //////////////////////////////
    else if(file_flag == 1){ //FSM_state1
        F_sum     = 0;
        F_index   = 0;
        F_length  = UCA0RXBUF;   // length LSB
        file_flag = 2; // FSM_state2
    }


    /////////// File state FSM_state2 ///////////////
    else if(file_flag == 2){//FSM_state2
        F_length |= ((unsigned int)UCA0RXBUF << 8); // length MSB
        rx_addr = fs.next_free;

        if (rx_addr < FILES_REGION_START || rx_addr > FILES_REGION_END){
            UCA0TXBUF = '!';
            file_flag = 0;  // not legal adders range, clear flag
            return;
        }

        FLASHConfig();


        if ((rx_addr & (FILE_SEG_SIZE - 1)) == 0) { // Start write WITHOUT erasing when not at segment boundary
            flash_write_init((int)rx_addr);      // erase + WRT at new segment
        } else {
            flash_wrt_no_erase((int)rx_addr);    // WRT only, no erase
        }
        current_seg_base = rx_addr & ~(FILE_SEG_SIZE - 1);
        file_flag = 3; // go to FSM_state3
    }


    /////////// File state FSM_state3  //////////////
    else if(file_flag == 3){    //FSM_state3
        unsigned char b = UCA0RXBUF;

        Char2Flah(b);        // write char to flash
        F_index++;           // count bytes

        if ((unsigned int)FLASH_ptr > (FILES_REGION_END + 1)){
            dis_FLASHWrite();
            UCA0TXBUF = '!'; // error send to PC
            file_flag = 0; // ptr not in adders range, clear flag
            return;
        }

        // when crossing a segment boundary, open the next segment (erase it once)
        flash_advance_segment_if_needed();

        F_sum = (F_sum + b);
        if (F_index == F_length){ // we get all characters from PC
            file_flag = 4;      // go to  FSM_state4
        }
    }
    //////////////// File state  FSM_state4  /////////////////
    else if(file_flag == 4){// FSM_state4

        F_chk = UCA0RXBUF;
        dis_FLASHWrite(); // stop write to Flash

        if(((F_sum + F_chk) & 0xFF) == 0){
            UCA0TXBUF = '$';     // ACK
            LPM0_EXIT;
            file_flag = 0;    // clear flag

            FileEntry *e = &fs.entries[rx_slot - 1];
            if (!e->in_use) fs.num_files++;
            e->in_use = 1;
            e->type   = F_type;
            e->start  = rx_addr;
            e->size   = F_index;

            unsigned int nf = rx_addr + F_index;
            if (nf > fs.next_free) fs.next_free = nf;
            fs_store();// store fs to flash memory

        }
        else{
            UCA0TXBUF = '!';     // NACK
            file_flag = 3;      // not acknowledged try to send it again go to FSM_state3 , 2
            F_index = 0;
        }
    }

    //////////////////  end of File state FSM /////////////////////
    ////////////////////  Script Name of File Sate  ///////////////
    else if(get_Name_flag == 1){
        ScriptName[S_index++] = UCA0RXBUF;
        if (S_index>= 16){
                get_Name_flag = 0;
                LPM0_EXIT;
                S_index = 0 ;
                SName_flag = 0;
                }
        }


    /////////////////// Main States  ////////////////////////////
    else
    {
        if(UCA0RXBUF == '0' ){
            state = state0;
        }
        if(UCA0RXBUF == '1' ){
            state = state1;
            detec_state = detect_sleep ;
        }
        else if (UCA0RXBUF == '2'){
            state = state2;
            Telemeter_state = Telemeter_sleep ;
        }
        else if (UCA0RXBUF == '3'){
            state = state3;
            light_detector_state = LDR_sleep ;
        }

        else if (UCA0RXBUF == '4'){//calibration
            lcd_clear() ;
            lcd_puts("state4");
            state = state4;        }

        else if (UCA0RXBUF == '5'){
            state = state5;
            FileMode_state = FileMode_sleep;
        }

        else if (UCA0RXBUF == '6'){//calibration
            state = state6;
        }
//////////////////// file slots  /////////////////////////////
        else if (UCA0RXBUF == 'A'){
            //FileMode_state = load_file1 ;
            rx_slot = 1;
        }
        else if (UCA0RXBUF =='B' ){
            //FileMode_state = load_file2 ;
            rx_slot = 2;
        }
        else if (UCA0RXBUF == 'C'){
            //FileMode_state = load_file3 ;
            rx_slot = 3;
        }
        else if (UCA0RXBUF == 'D'){
            //FileMode_state = load_file1 ;
            rx_slot = 4;
        }
        else if (UCA0RXBUF =='E' ){
            //FileMode_state = load_file2 ;
            rx_slot = 5;
        }
        else if (UCA0RXBUF == 'F'){
            //FileMode_state = load_file3 ;
            rx_slot = 6;
        }
        else if (UCA0RXBUF == 'G'){
            //FileMode_state = load_file1 ;
            rx_slot = 7;
        }
        else if (UCA0RXBUF =='H' ){
            //FileMode_state = load_file2 ;
            rx_slot = 8;
        }
        else if (UCA0RXBUF == 'I'){
            //FileMode_state = load_file3 ;
            rx_slot = 9;
        }
        else if (UCA0RXBUF == 'J'){
            //FileMode_state = load_file3 ;
            rx_slot = 10;
        }
        /////// File Type ///////////
        else if(UCA0RXBUF == 'T'){
            F_type = FT_TEXT;
            //FileMode_state= FileModeText; // TEMP MAY CAHNGE!!!!!!!!!!!!
            if(SName_flag==0)file_flag = 1; // go to FSM_state1
            else if (SName_flag==1){
                FileMode_state= FileModeText;
                //get_Name_flag = 1; // temp !!!!!!!!!!

            }
            if(first_FileUp == 0&&SName_flag==0){
                if ((int)fs.num_files == 0)fs_load();
                first_FileUp = 1;
            }
        }
        else if (UCA0RXBUF == 'S'){
            F_type = FT_SCRIPT;
            if(SName_flag==0){
                file_flag = 1;  // go to FSM_state1
                FileMode_state_temp = FileModeScript;// TEMP MAY CAHNGE!!!!!!!!!!!!
                if(first_FileUp == 0){
                    if ((int)fs.num_files == 0)fs_load();

                    first_FileUp = 1;
                }
                }
            if(SName_flag == 1){
                get_Name_flag = 1;
                FileMode_state = FileModeScript;
                }

        }

        /////// Script mode to execute ///////////
        else if (UCA0RXBUF == 'N'){
             SName_flag = 1 ;
             S_index = 0 ;
        }

        else if(UCA0RXBUF == 'K')
        {
            cal_tx_flag = 1;
            cal_tx_phase = 0;
            cal_tx_sub = 0;
            cal_tx_idx = 0;
            cal_tx_sensor = 1;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = 'K';  // Acknowledge to PC
            }



        else if(UCA0RXBUF == 'X')
                {
            SourcesObjects_state = SourcesObjects_awake ;
            //Sourc_Obj_flag = 0 ;

                    }
/////////////////////////////////////////////////////////////////////////
        else if (UCA0RXBUF == 's'){
            detec_state = detect_start;
        }
        else if(UCA0RXBUF == 'l'){
            light_detector_state = LDR_awake;
        }
        else if(UCA0RXBUF == 't'){
            Telemeter_state = Telemeter_start;
            Tel_angle_idx = 0;
            Tel_flag = 1;
        }

        switch(lpm_mode){
            case mode0: LPM0_EXIT; break;
            case mode1: LPM1_EXIT; break;
            case mode2: LPM2_EXIT; break;
            case mode3: LPM3_EXIT; break;
            case mode4: LPM4_EXIT; break;
        }
    }
}
