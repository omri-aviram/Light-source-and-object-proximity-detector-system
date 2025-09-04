#ifndef _halGPIO_H_
#define _halGPIO_H_

#include  "../header/bsp_msp430x2xx.h"          // private library - BSP layer
#include  "../header/app.h"         // private library - APP layer
//extern enum LightSourcesObjects SourcesObjects_state;
extern enum FileMode FileMode_state ;
extern enum objcet_func detec_state;
extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern enum TelemeterState Telemeter_state; // global variable
extern enum LDRstate light_detector_state; //global variable
extern enum LightSourcesObjects SourcesObjects_state; //global variable

unsigned int flag_state3 ;
extern unsigned int Meas1, Meas2;
//////////// ecoh Variable ////////
extern int t_diff;
extern int measure_result[4];
extern unsigned char diff_bytes[2];
extern unsigned int diff_tx_i ;
extern unsigned char diff_flag ;
extern int initial_engal;
extern int dig;
//////////// LDR Variables ////////
extern int LDR_samp1;
extern int LDR_samp2;
extern int avg_LDRsample;
extern int LDR_sample;
//extern int LDR_sample;
unsigned char LDR_bytes[2];
//extern unsigned int LDR_tx_i;
extern unsigned int LDR_flag;
//////////// Telemeter Variables ////////
extern char Tel_angle_rx[4];
extern int Tel_scanFlag;
extern unsigned int Tel_tx_i;
extern unsigned char Tel_bytes[2] ;
extern int tele_angle_int ;
//////////////////////// Light Sources Objects ///////////////////////

extern unsigned int Sourc_Obj_flag ;
////////////////////system configuration/////////////////
extern void sysConfig(void);
extern unsigned char readSWs(void);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void incLEDs(char);
extern void enable_interrupts();
extern void disable_interrupts();
extern __interrupt void PBs_handler(void);

//////////////////// File_Mode/////////////////
#define MAX_FILES 3
#define MAX_FILENAME_LENGTH 8 // including "\0"

extern int F_length ;
extern char ScriptName [16];
extern int SName_flag;

#define FILE_SEG_SIZE       512


////////////////////// NEW ////////////////////////////
#define FS_MAX_FILES        10
#define FILES_REGION_END    (FILES_REGION_START + FILES_REGION_SIZE - 1)
#define FILE_SEG_SIZE       512
#define FILES_REGION_SIZE   0X800// IN DEC                ;BEFOR: 2048
#define FILES_REGION_START  0XF600//0xF000
#define FS_INFO_ADDR   0x1040   // Segment C
#define FILE_SEG_SIZE 512
extern enum Ftype F_type;

typedef struct {
    unsigned int  start;   // Flash start adders
    unsigned int  size;    // NUM OF BYETS INCLOD EOF
    unsigned char type;    // FT_SCRIPT / FT_TEXT
    unsigned char in_use;  // 0/1 0 not use  1 used
} FileEntry;

typedef struct {
    unsigned char num_files;   // number of file in MCU
    unsigned int  next_free;   // next free adders
    FileEntry     entries[10]; // max 10 file
    unsigned char crc;         // checksum 8-bit
} FSHeader;

extern void load_fs();
extern FSHeader fs;
extern int first_FileUp ;
extern volatile unsigned char pb_event;
//////////////////// Flash section ////////////////////////
extern void Char2Flah(char CH );


////////////////////////////  calibration ////////////////////////
#define CAL_POINTS 10
//#define LUT_SIZE   50
#define CAL1_ADDR  0x1000
#define CAL2_ADDR  0x1080 // no enofe sens
extern void cal_load(void);
extern void cal_build_lut(void);
//extern unsigned int lut1[LUT_SIZE];
//extern unsigned int lut2[LUT_SIZE];
extern unsigned int cal1[CAL_POINTS];
extern unsigned int cal2[CAL_POINTS];
extern unsigned char cal_crc8(const unsigned char *p, unsigned int len);

////////////////////  LCD  ///////////////////////////////
#define FOURBIT_MODE    0x0
#define LCD_MODE        FOURBIT_MODE

void lcd_init();
void lcd_cmd(unsigned char comm);
void lcd_strobe ();
void lcd_data(unsigned char c);
void lcd_puts(const char *s);

///////////////////////// our function ///////////////////////
////////  Servo  /////////////////
extern void PWMGEN_Servo(int feq);
void PWM_OFFServo();
///////  PWM senc trigger //////////////
void PWMTriggerGEN();
/////// Timer/////////////////
extern void enable_timerA0();
extern void disable_timerA0();
extern void PWNTriggerGEN();
extern void disable_timerA1();
void PWMTriggerDisable();
//////// ADC 10 /////////////
extern void enable_samp();
extern void disable_ADC();
extern void init_LDR1();
extern void init_LDR2();
unsigned char get_sample();
///////  FLASH  //////////////
extern void dis_FLASHWrite();
extern void flash_write_init(int addr);
extern void fs_load(void);
extern void fs_store(void);
static unsigned char fs_crc8_calc(const unsigned char *p, unsigned int len);
static inline const FSHeader* fs_map(void);
extern int fs_find_index_by_name(const char* arr_name);
#define FS_INFO_ADDR   0x1040

////////////////////// script execution function
void clear_timerA0();

///////////////////////// UART ///////////////////
void send_stop();
//////////// state 5 //////////////////////////////

void LDR_object_measure();
///////////////////////////////////////////////////////////////


#endif







