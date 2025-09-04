#ifndef _api_H_
#define _api_H_

#include  "../header/halGPIO.h"     // private library - HAL layer

//void detect_obj();
//void engine_scan(int angle1,int angle2);
//int get_distance();


////////// state 1 /////////////////////

void detect_obj();
void get_distance_measure();
void get_distance();

void get_LDR_measure();
void light_detector_mode();

void engine_scan(int angle1,int angle2,int choose_scan);

////////// state 2 ///////////////////
void Telemeter_mode();
//void get_Tel_angle();
void Tel_distance();
void get_Tel_angle(int angle_raw);
////////// state 3 ///////////////////
void LDR_SAMP(int LDR_choose);

////////// state 4 ///////////////////
void Light_Sources_Objects();

//////// state 5 /////////////////////
void write2flash(int file_num);
void inc_lcd(int x);
void rra_lcd(int x);
void set_delay(int delay);
void servo_deg(int p);

////////////////////////////////  LDR calibration mode
void LDR_calibration(void);
//void cal_build_lut(void);
void cal_store(void);
//void cal_load(void);
static unsigned char cal_crc8(const unsigned char *p, unsigned int len);
#endif
