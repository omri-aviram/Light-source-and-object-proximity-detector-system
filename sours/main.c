#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;
unsigned int Meas1, Meas2;


void main(void){




 state = state0;  // start in idle state on RESET
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();

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

