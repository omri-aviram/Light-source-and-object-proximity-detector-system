#ifndef _app_H_
#define _app_H_

enum FileMode{FileModeText,FileModeScript,FileMode_sleep};
enum LightSourcesObjects{SourcesObjects_sleep,SourcesObjects_awake};
enum objcet_func{detect_sleep, detect_start};
enum TelemeterState{Telemeter_sleep, Telemeter_start};
enum LDRstate{LDR_sleep,LDR_awake};
enum FSMstate{state0,state1,state2,state3,state4,state5,state6}; // global variable
enum SYSmode{mode0,mode1,mode2,mode3,mode4,mode5,mode6}; // global variable
enum Ftype { FT_SCRIPT = 1, FT_TEXT = 2 };
///////////  calibration section /////////////////
#define CAL_POINTS 10
#define LUT_SIZE   50
void LDR_calibration_mode(void);
void cal_load(void);
void cal_store(void);
void cal_build_lut(void);



#endif







