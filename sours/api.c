#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer

static inline void uart_send_byte(unsigned char b){
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = b;
}
static inline void uart_send_u16(unsigned int v){
    uart_send_byte((unsigned char)(v & 0xFF));
    uart_send_byte((unsigned char)((v >> 8) & 0xFF));
}
/////////////////////////////////////////////////    our function      /////////////////////////////////////////////////
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
int index ,val ;
char opcode [3] = {'0','0','\n'};
char   arg1 [3] = {'0','0','\n'};
char   arg2 [3] = {'0','0','\n'};
int  opcode_int = 0;
int    arg1_int = 0;
int    arg2_int = 0;

//-------------------------------------------------------------
//                      state1
//-------------------------------------------------------------


void detect_obj(){
    detec_state = detect_sleep;

    while(state == state1){
        if (detec_state == detect_sleep){
            enterLPM(lpm_mode);
        }
        else if (detec_state == detect_start){
            int angel1 = 0;
            int angel2 = 180;
            engine_scan(angel1,angel2,1);
            detec_state = detect_sleep ;
        }
    }
}

//-------------------------------------------------------------
//                      state2
//-------------------------------------------------------------
void Telemeter_mode(){
    while(state == state2){
            tele_angle_int =0;
            if (Telemeter_state == Telemeter_sleep){
                enterLPM(lpm_mode);
            }
            else if (Telemeter_state == Telemeter_start){
                //enterLPM(lpm_mode);
               tele_angle_int = atoi(Tel_angle_rx); // i change that
                get_Tel_angle(tele_angle_int);
                Tel_scanFlag = 1;
                while(Telemeter_state == Telemeter_start && state == state2){
                    get_distance();
                    __delay_cycles(100000);

                }
                Tel_scanFlag = 0;
                Telemeter_state = Telemeter_sleep ;

            }
        }
    }





//-------------------------------------------------------------
//                      state4
//-------------------------------------------------------------


void Light_Sources_Objects(){


    while(state == state4){

                if (SourcesObjects_state == SourcesObjects_sleep){
                    lcd_clear() ;
                    lcd_home();
                    lcd_puts("Sources&Objects");
                    enterLPM(lpm_mode);
                }
                else if (SourcesObjects_state == SourcesObjects_awake){

                    //Sourc_Obj_flag = 1 ;
                    engine_scan(0,180,3);
                    //Sourc_Obj_flag = 0 ;

                    SourcesObjects_state = SourcesObjects_sleep ;

                }
            }



    //LightSourcesObjects = SourcesObjects_sleep;
    //while(state == state4){


}



//-------------------------------------------------------------
//                      state5
//-------------------------------------------------------------
//FileMode{FileModeText,FileModeScript};
void File_Mode(){
    int script_num = 0;
     while(state == state5){
        switch(FileMode_state){
        case FileMode_sleep:
            enterLPM(lpm_mode);
            break;

        case FileModeText:
            //enterLPM(lpm_mode);
            Text2Lcd();
            break;

        case FileModeScript:
            enterLPM(lpm_mode);
            load_fs();   // with this line we cane add more file on top of the initial file in flash !
            script_num = fs_find_index_by_name(ScriptName);
            uart_send_byte('$');// sending to PC we are Script Mode execution
            play_script_by_index(script_num);
            uart_send_byte(0xFF);
            uart_send_u16(1); // sending to PC we are ending Script Mode execution
            break;

            }
     }

}


////////  Execute Script /////////////////

//void Ex_Script(int idx){

void play_script_by_index(unsigned int idx){

    int sleep_state_flag = 0;
    const FSHeader* h = fs_map();
    if(!h) return;
    if(idx >= h->num_files) return;
    const FileEntry* e = &h->entries[idx];
    if(!e->in_use) return;

    unsigned int end = e->start + e->size;
    char* read_ptr = (char*)(unsigned int)(e->start + 16); // first 16 char is file name

    //while((unsigned int)read_ptr < end && *read_ptr) read_ptr++;
    //if((unsigned int)read_ptr < end) read_ptr++;

    while(sleep_state_flag == 0 && (unsigned int)read_ptr < end){
        opcode[0] = *read_ptr++;
        if((unsigned int)read_ptr >= end) break;
        opcode[1] = *read_ptr++;
        //send_opcode();
        __delay_cycles(1250000);

        if((unsigned int)read_ptr >= end) break;
        if(*read_ptr != 10){
            arg1[0] = *read_ptr++;
            if((unsigned int)read_ptr >= end) break;
            arg1[1] = *read_ptr++;
            if((unsigned int)read_ptr >= end) break;
            if(*read_ptr != 10){
                arg2[0] = *read_ptr++;
                if((unsigned int)read_ptr >= end) break;
                arg2[1] = *read_ptr++;
                if((unsigned int)read_ptr < end) read_ptr++;
            }
            else {
                read_ptr++;
            }
        }
        else {
            read_ptr++;
        }

        opcode_int = atoi(opcode);
        arg1_int = strtol(arg1, NULL, 16);
        arg2_int = strtol(arg2, NULL, 16);

        switch(opcode_int){
            case 1:
                inc_lcd(arg1_int);
                break;
            case 2:
                dec_lcd(arg1_int);
                break;
            case 3:
                rra_lcd(arg1_int);
                break;
            case 4: // set delay value

                set_delay(arg1_int);
                break;
            case 5:
                lcd_clear();
                break;
            case 6:
                lcd_clear();
                lcd_puts("Servo deg:");
                //lcd_puts(arg1);
                servo_deg(arg1_int);
                //uart_send_byte(0xFF);
                //uart_send_u16(1);
                get_distance_measure();
                //uart_send_byte(0xFF);
                //uart_send_u16(1);
                __delay_cycles(100000);
                break;
            case 7:
                lcd_clear();
                lcd_puts("Scanning");
                //lcd_cursor2();
                //lcd_puts("Environment");
                engine_scan(arg1_int, arg2_int, 1);
                break;
            case 8:
                lcd_clear();
                lcd_puts("Sleep stat");
                sleep_state_flag = 1;
                break;
        }
    }
    FileMode_state = FileMode_sleep;
    SName_flag = 0 ;
}





////////////////////////////////////////////////////////////



//-------------------------------------------
//               READ TEXT FROM FLASH
//-------------------------------------------
void Text2Lcd(void){
    const FSHeader* h = fs_map();
    int idx[10];
    int n = 0;
    int i;

    if(!h){
        lcd_clear();
        lcd_puts("No FS");
        lcd_new_line();
        lcd_puts("PB1 Back");
        while (state == state5 && FileMode_state == FileModeText){
            enterLPM(lpm_mode);
            if (pb_event == 2){ pb_event = 0; break; }
        }
        return;
    }

    for(i = 0; i < h->num_files && i < 10; i++){
        const FileEntry* e = &h->entries[i];
        if(!e->in_use) continue;
        if(e->type != FT_TEXT)  continue;
        idx[n++] = i;
    }

    if (n == 0){
        lcd_clear();
        lcd_puts("No TXT files");
        lcd_new_line();
        lcd_puts("PB1 Back");
        while (state == state5 && FileMode_state == FileModeText){
            enterLPM(lpm_mode);
            if (pb_event == 2){ pb_event = 0; break; }
        }
        return;
    }

    int sel = 0;
    int mode = 0;
    int off = 0;

    while (state == state5 && FileMode_state == FileModeText){
        if (mode == 0){
            const FileEntry* e1 = &h->entries[idx[sel]];
            const char* name1 = (const char*)(unsigned int)e1->start;
            char line1[17]; int j;

            for (j=0; j<16; j++){
                char c = name1[j];
                if (c == 0) { line1[j] = ' '; continue; }
                if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
                line1[j] = c;
            }
            line1[16] = 0;

            char line2[17]; int k;
            if (n >= 2){
                const FileEntry* e2 = &h->entries[idx[(sel+1)%n]];
                const char* name2 = (const char*)(unsigned int)e2->start;
                for (k=0; k<16; k++){
                    char c = name2[k];
                    if (c == 0) { line2[k] = ' '; continue; }
                    if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
                    line2[k] = c;
                }
                line2[16] = 0;
            } else {
                for (k=0; k<16; k++) line2[k] = ' ';
                line2[16] = 0;
            }

            lcd_clear();
            lcd_puts(line1);
            lcd_new_line();
            lcd_puts(line2);

            enterLPM(lpm_mode);
            if (pb_event == 1){
                pb_event = 0;
                sel++;
                if (sel >= n) sel = 0;
            } else if (pb_event == 2){
                pb_event = 0;
                mode = 1;
                off = 0;
            }
        } else {
            const FileEntry* e = &h->entries[idx[sel]];
            unsigned int data_start = e->start + 16;
            int data_len = (e->size > 16) ? (e->size - 16) : 0;

            char *p0 = (char*)(unsigned int)(data_start + off);
            char *p1 = (char*)(unsigned int)(data_start + off + 16);
            int k;

            lcd_clear();
            for (k=0; k<16; k++){
                char c = (off + k < data_len) ? p0[k] : ' ';
                if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
                lcd_data((unsigned char)c);
            }
            lcd_new_line();
            for (k=0; k<16; k++){
                char c = (off + 16 + k < data_len) ? p1[k] : ' ';
                if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
                lcd_data((unsigned char)c);
            }

            enterLPM(lpm_mode);
            if (pb_event == 1){
                pb_event = 0;
                if (off + 32 < data_len) off += 32;
                else off = 0;
            } else if (pb_event == 2){
                pb_event = 0;
                mode = 0;
            }
        }
    }
    SName_flag = 0; // CLAER NFLAG
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//       GET Telemeter angle
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//void get_Tel_angle(){
    //int angle;
    //tele_angle_int = atoi(Tel_angle_rx);
void get_Tel_angle(int angle_raw){
    int angle;
    angle = (angle_raw*10) + 100 ;
    TIMERA1config(angle);
    PWMGEN_Servo(angle);
    __delay_cycles(500000); // temp!
    disable_timerA1();

}

/////////////////////////  script execution function  ////////////////////////




//  ~~~~~~~~~~~~~~~~~~~~~  counting up on LCD ~~~~~~~~~~~~~~~~~~~~~
void inc_lcd(int x){
  int upcounter = 0;
  char inc_counter_str[8];
  char x_str[10];
  TIMERA0config();
  lcd_init();
  clear_timerA0();;
  //sprintf(x_str, "%d", x);
  //lcd_clear();
  //lcd_puts("inc_lcd:");
  //lcd_puts(x_str);

  while(upcounter <= x)
  {
      enable_timerA0(); //enable the timer with delay of d[msec]
      enterLPM(lpm_mode);
      disable_timerA0();
      sprintf(inc_counter_str, "%d", upcounter);
      //lcd_goto(0x40);
      //lcd_puts (clear_Buff);
      //lcd_goto(0x40);
      lcd_clear();
      lcd_puts(inc_counter_str);      //print
      upcounter ++;
  }
  lcd_clear();
  disable_timerA0();

}



//  ~~~~~~~~~~~~~~~~~~~~~  counting down on LCD ~~~~~~~~~~~~~~~~~~~~~

void dec_lcd(int x){
  int downcounter = x;
  char dec_counter_str[10];
  char x_str[10];
  TIMERA0config();
  lcd_init();
  clear_timerA0();
  //sprintf(x_str, "%d", x);
  //lcd_clear();
  //lcd_puts("dec_lcd:");
  //lcd_puts(x_str);
  //lcd_new_line();
  while(downcounter >= 0)
  {
      enable_timerA0(); //enable the timer with delay of d[msec]
      enterLPM(lpm_mode);
      disable_timerA0();
      sprintf(dec_counter_str, "%d", downcounter);
      //lcd_new_line();
      //lcd_puts (clear_Buff);
      //lcd_new_line();
      lcd_clear();
      lcd_puts(dec_counter_str);      //print
      downcounter --;
  }
  lcd_clear();
  disable_timerA0();
}


//  ~~~~~~~~~~~~~~~~~~~~~  Rotate right char onto LCD ~~~~~~~~~~~~~~~~~~~~~
void rra_lcd(int x){
    char Char2Rotate [2] = {'0','\0'};
    Char2Rotate[0] = x + '0'; // from int -> char
    int counter=0;
    TIMERA0config();
    lcd_init();
    lcd_clear();

    while (counter < 32){
        enable_timerA0(); //enable the timer with delay of d[msec]
        enterLPM(lpm_mode);
        disable_timerA0();


        lcd_cursor_left();
        lcd_puts(" ");
        lcd_puts(Char2Rotate);


        if (counter == 15){
            lcd_cursor_left();
            lcd_puts(" ");
            lcd_new_line();
        }
        else if(counter == 31){
            lcd_clear();
            disable_timerA0();
        }
        counter++;
    }
}

//  ~~~~~~~~~~~~~~~~~~~~~  Set Delay ~~~~~~~~~~~~~~~~~~~~~
void set_delay(int delay){
  d=delay;
  char delay_str[10];
  lcd_clear();
  lcd_puts("delay is:");
  sprintf(delay_str, "%d", delay);
  lcd_puts(delay_str);
}

//  ~~~~~~~~~~~~~~~~~~~~~  servo_deg ~~~~~~~~~~~~~~~~~~~~~
void servo_deg(int p){
    //send distance to PC
    tele_angle_int = p;
    get_Tel_angle(p);
    //meas_and_send_distance();
    __delay_cycles(250000);
    //send angle to PC
    //calc_and_send_angle(p);
    Tel_scanFlag = 1;
    get_distance();// maybe get_distance_measure
    Tel_scanFlag = 0;

}




//-------------------------------------------------------------
//                      ULTRASONIC + SERVO
//-------------------------------------------------------------



//TIMERA1config();
// PWMGEN_Servo(initial_angle);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//        servo scan function
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void engine_scan(int angle1,int angle2,int choose_scan){
    int stop_angle = 2000;
    int initial_angle = 100;
    TIMERA1config(); // Continuous mode + SMCLK
    TIMERA0config();    // upDown + SMCLK + Div_8
    dig = angle1 ;
    d = 25;         // delay is 250 ms , 25 6 is 60ms 1 is 10 ms!!!!!!!!!!!!!!!!maybe needs to be  100 ms or 500ms
    stop_angle    = (angle2 * 10) + 40;//130     // up to 180 deg  hall 100
    initial_angle = (angle1 * 10) + 400; //400         // initialize   hall 400
    PWMGEN_Servo(initial_angle);
    __delay_cycles(60000); //100 0000
    {
        unsigned int n1 = (unsigned int)((angle2>=angle1)?(angle2-angle1):(angle1-angle2));
        uart_send_byte(0xFF);
        uart_send_u16(n1>>1);//n1>>1 ->180/2
    }
    while(initial_angle < stop_angle){
        enable_timerA0();
        enterLPM(lpm_mode);
        disable_timerA0();
        initial_angle += 16;// 17 , 16
        __delay_cycles(60000);
        PWMGEN_Servo(initial_angle);
        __delay_cycles(50000);
        if (choose_scan == 1){
            get_distance_measure(); //the TimerA1 sends the measurement to UART TxBuffer
        }
        else if (choose_scan == 2){
            get_LDR_measure();
            __delay_cycles(100000);
        }
        else if (choose_scan == 3){
            LDR_object_measure();
            __delay_cycles(100000);
        }
        //initial_angle += 18;// 9
        dig +=2 ;// 1
        TIMERA0config();
        PWMTriggerDisable(); // need to check more! see if it work


        }

    __delay_cycles(100000);
    __delay_cycles(1500000);
    PWMGEN_Servo(1000); // INIT to 90 deg
    __delay_cycles(1500000);
    
    disable_timerA0();
    disable_timerA1();
   d = 50; // d go back to default value

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//        measure distance
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void get_distance(){
    PWMTriggerGEN(); // work good
    TIMER_USconfig(); // making timer 1 input Capture good
    enterLPM(lpm_mode);
    disable_timerA1();
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//        get measuremants
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void get_distance_measure(){
        //PWM_OFFServo();
        get_distance();
}

//-------------------------------------------------------------
//                      state3
//-------------------------------------------------------------

void light_detector_mode(){
    //light_detector_state = LDR_sleep;
    while (state == state3){
    if (light_detector_state == LDR_sleep){
        enterLPM(lpm_mode);
    }
    else if (light_detector_state == LDR_awake){
        engine_scan(0,180,2);
        light_detector_state = LDR_sleep;
        //state = state0;
        }
    }
}


void get_LDR_measure(){
    unsigned char v1, v2;
    ADC_init();

    //init_LDR1();
    //enable_samp();
    //enterLPM(mode0);
    //disable_ADC();
    LDR_SAMP(1);
    v1 = LDR_sample;//get_sample();
    //init_LDR2();
    //enable_samp();
    //enterLPM(mode0);
    //disable_ADC();
    LDR_SAMP(2); //takes 4 samples and take the mean val
    v2 = LDR_sample;//get_sample();
    //LDR_SAMP(1); //sampling LDR1 measurement
    LDR_bytes[0] = v1 ;
    LDR_bytes[1] = v2 ;
    send_samp();
    __delay_cycles(50000);




    //            LDR_SAMP(1);

    //s1 = LDR_sample;
    //LDR_SAMP(2); //sampling LDR2 measurement
    //send_samp();LDR_bytes[0]
}


/*     LDR_SAMP  change !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void LDR_SAMP(int LDR_choose){
            int i;
            ADC_init(); // Initialing


            if(LDR_choose == 1 ){

                init_LDR1();
            }
            else if (LDR_choose == 2 ){

                init_LDR2();

            }
            LDR_sample = 0;
            for (i = 0;i<4;i++){
                enable_samp();
                enterLPM(mode0);
                disable_ADC();
                LDR_sample+=get_sample();
            }
            LDR_sample = LDR_sample>>2;//mean of samples to reduce noise


         if(LDR_choose == 1 ){

             LDR_samp1 = LDR_sample;
         }
         else if (LDR_choose == 2 ){
             LDR_samp2=LDR_sample;
             LDR_sample_avg = (LDR_samp2 + LDR_samp1)>>1; // get average
         }
}
*/
void LDR_SAMP(int LDR_choose){
            int i;
            ADC_init(); // Initialing


            if(LDR_choose == 1 ){

                init_LDR1();
            }
            else if (LDR_choose == 2 ){

                init_LDR2();

            }
            LDR_sample = 0;
            for (i = 0;i<4;i++){
                enable_samp();
                enterLPM(mode0);
                disable_ADC();
                LDR_sample+=(unsigned char)get_sample();//(unsigned int)
            }
            LDR_sample = LDR_sample>>2;//mean of samples to reduce noise


        /* if(LDR_choose == 1 ){

             LDR_samp1 = LDR_sample;
         }
         else if (LDR_choose == 2 ){
             LDR_samp2=LDR_sample;
             LDR_sample = (LDR_samp2 + LDR_samp1)>>1; // get average
         }

         */


}

void LDR_object_measure(){

    get_distance_measure();
    __delay_cycles(100000);
    Sourc_Obj_flag = 1 ;
    get_LDR_measure();
    Sourc_Obj_flag = 0 ;
}




////////////////// NEW2////////////////////////////////

static inline const FSHeader* fs_map(void){ // return ptr to FSHeader
    return (const FSHeader*)(unsigned int)FS_INFO_ADDR;/* segmant C 0x1040 */
}
//***********************************************************************
///////////////////////// CALIBRATION FUNCTION /////////////////////////
//***********************************************************************
// SAMPLE IN FLASH :
// BYET ADDRES|  VALUE
//     0      | C 1\2 --SELECT LDR for calibration
//     1      | NUM OF CAL POINT = 10
//     2      | CHECK SUM VAL
//     3      | 0
//  4 : 23    | sample calibration point 2 byet for 1 sampel
//  26 : 64   | 0XFF <- VALUE OF EMPTY FLASH




////////////////////////////////  LDR calibration mode ////////////////////////////////////





void LDR_calibration(void){
    ADC_init();
    int step = 0;
    char step_cahr[3];
    cal_load();
    lcd_clear();
    lcd_puts("Cal LDR PB0");
    lcd_new_line();
    lcd_puts("PB1 Exit");

    while (state == state6){
        enterLPM(lpm_mode);

        if (pb_event == 2){ // PB1 IS preset
            pb_event = 0;
            state = state0;
            break;
        }

        if (pb_event == 1){// PB0 IS preset
            pb_event = 0;


        if (step < CAL_POINTS){
            unsigned int s1, s2;

            //init_LDR1();
            //enable_samp();
            //enterLPM(lpm_mode);
            //disable_ADC();
            LDR_SAMP(1);

            s1 = LDR_sample;

            while(1){
                lcd_clear();
                lcd_puts("press for LDR2");
                enterLPM(lpm_mode);
                if (pb_event == 1){// PB0 IS preset
                    pb_event = 0;
                    break;
                }
            }

            //init_LDR2();
            //enable_samp();
            //enterLPM(lpm_mode);
            //disable_ADC();
            LDR_SAMP(2);
            s2 = LDR_sample;

            if (step < CAL_POINTS){
                cal1[step] = s1;
                cal2[step] = s2;
                step++;
                lcd_clear();
                lcd_puts("Saved Samp num:");
                if(step == 10)lcd_new_line();
                sprintf(step_cahr, "%d", step);
                lcd_puts(step_cahr);

                }
            if (step >= CAL_POINTS){
                   cal_store();
                   lcd_cmd(0x1);
                   lcd_puts("Calibration saved");
                   lcd_new_line();
                   lcd_puts("PC: press 'Fetch Cal'");
                   state = state0;
                      }
                }
            }
        }
    }

/////////////////////////////// store calibration samples ///////////////////////////////////

void cal_store(void){  // cal_store Saves calibration samples to FLASH in a structured format at the start of the calibration block

    unsigned char buf[64]; // temp buffer

    unsigned char *p;

    unsigned int i;

    memset(buf, 0xFF, sizeof(buf));

    buf[0] = 0xC1;      // mark as calibration for LDR 1
    buf[1] = CAL_POINTS;  //     NUMBER OF CLIBRATION POINT IS 10
    buf[2] = 0;         // CLEAR CHECK SUM PLACE
    buf[3] = 0;         // not use we dont wont garbage
    p = &buf[4];        // start of samples
    for (i=0;i<CAL_POINTS;i++){
        p[0] = (unsigned char)(cal1[i] & 0xFF); //  SAPM LSB
        p[1] = (unsigned char)((cal1[i] >> 8) & 0xFF);// SAMP MSB
        p += 2;
    }
    buf[2] = cal_crc8(buf, 64); // GEY CHECK SUM
    flash_write_init((int)CAL1_ADDR); // start flash write
    for (i=0;i<64;i++) Char2Flah(buf[i]); // write buffer to flash
    dis_FLASHWrite(); // close flash
    /////////// do the same to LDR2
    memset(buf, 0xFF, sizeof(buf));
    buf[0] = 0xC2;
    buf[1] = CAL_POINTS;
    buf[2] = 0;
    buf[3] = 0;
    p = &buf[4];
    for (i=0;i<CAL_POINTS;i++){
        p[0] = (unsigned char)(cal2[i] & 0xFF);
        p[1] = (unsigned char)((cal2[i] >> 8) & 0xFF);
        p += 2;
    }
    buf[2] = cal_crc8(buf, 64);
    flash_write_init((int)CAL2_ADDR);
    for (i=0;i<64;i++) Char2Flah(buf[i]);
    dis_FLASHWrite();
}









  
  
  

