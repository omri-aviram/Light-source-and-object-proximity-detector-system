#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer
 // const unsigned char *src = (const unsigned char*)FS_INFO_ADDR; // initial adders of fill manager in flash
 // FSHeader cand;
 // memcpy(&cand, src, sizeof(FSHeader));
  //while(1);
//enum objcet_func detec_state;
//enum LDRstate light_detector_state;
enum FSMstate state;
enum SYSmode lpm_mode;
unsigned int Meas1, Meas2;


void main(void){

    //detec_state = detect_start;

  //light_detector_state = LDR_awake;
 state = state0;  // start in idle state on RESET
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();
  //TIMERA1config();
 // PWMGEN_Servo(1000);

  //lcd_clear();
  //lcd_puts("Can't find TXT");
 // while(1);
  while(1){
	switch(state){

	  case state0:
	      lcd_clear() ;
	      lcd_puts("Main Menu");
	      get_Tel_angle(90);
          enterLPM(lpm_mode);
		break;
		 
	  case state1:
	      lcd_clear() ;
	      lcd_home();
	      lcd_puts("Object Detector");
	      lcd_new_line();
	      lcd_puts("Scan: 0 to 180");
	      detect_obj();
	      break;
		
		
	  case state2: // if we try to go to Telemeter 2 times we stuck
	      lcd_clear() ;
	      lcd_home();
	      lcd_puts("Telemeter");
	      Telemeter_mode();
		break;

	  case state3:
	      lcd_clear() ;
	      lcd_home();
	      lcd_puts("Light sources");
	      lcd_new_line();
	      lcd_puts("Scan: 0 to 180");
	      light_detector_mode();
	      break;

      case state4:
          lcd_clear() ;
          lcd_home();
          lcd_puts("Combo Mode");
          Light_Sources_Objects();


        break;

      case state5:
          lcd_clear() ;
          lcd_home();
          lcd_puts("File Mode");
          File_Mode();
        break;

      case state6:
          lcd_clear() ;
          lcd_home();
          LDR_calibration();
        break;



	}

  }

}

//TIMERA1config();
//PWMGEN_Servo(1000);
//// check's
//TIMERA1config();
//PWMGEN_Servo(1000);
//const unsigned char *src = (const unsigned char*)FS_INFO_ADDR; // initial adders of fill manager in flash
//FSHeader cand;
// memcpy(&cand, src, sizeof(FSHeader));
//  dec_lcd(62);
//  inc_lcd(62);
//rra_lcd(30);
//set_delay(100);
//rra_lcd(30);
//servo_deg(90);
//get_distance_measure();
//TIMERA0config();
//PWMTriggerGEN();
//TIMER_USconfig();
//disable_timerA1();
//engine_scan(0,180,1);
//PWMTriggerDisable();
//PWMTriggerGEN();
//engine_scan(45,135,1);
// PWMTriggerDisable();
//PWMTriggerGEN();
//lcd_clear();
//lcd_puts("Can't find TXT");
//lcd_new_line();
//lcd_puts("file in Flash");
//while(1);
//cal_store();
//LDR_calibration();
// ADC_init();
//init_LDR1();
//enable_samp();
//enterLPM(mode0);
//disable_ADC();
//Meas1 = get_sample();
//init_LDR2();
//enable_samp();
//enterLPM(mode0);
//disable_ADC();
//Meas2 = get_sample();
//while(1);
