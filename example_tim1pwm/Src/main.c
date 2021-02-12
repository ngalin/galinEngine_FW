/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
//11th Jan - replaced PA15/PB5 (EncA/EncB) GPIO interrupts with TIM2 Encoder code.

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_TIM_PERIOD 1200//57600// 600//1200
#define M_PI 3.14159265358979323846
#define I_MAX 300
#define ADC_CHANNELS 3
#define RISING 1
#define FALLING 0
#define HIGH 1
#define LOW 0
#define DEGREE_120 85
#define DEGREE_120_HIRES 21845//21760 //(85*256)
#define ADC_TIMEOUT 10e-6 //timeout for adc conversion event. Be mindful that we have a PWM freq of 30kHz, which is 33us between pulses.
//motor modes
#define STOPPED 0
#define TRACKING 1
#define DOING 2 //this badly named mode implies that the motor is tracking to a list of encoder positions, simulating engine operation
#define INIT 3 //initialise motor zero position using encoder and hall sensors

#define ADC_HALF_RANGE 32768
#define VCC_DIV_2 1.65
#define AMP_V_BIAS 1.559
#define AMP_GAIN 1.528

//#define ENC_TO_THETA 0.001963495 //for a single pole pair
#define ENC_TO_THETA 0.002748894 //for 14 pole pairs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void motor_error(void);
void motor_disable(void);
void motor_enable(void);
void motor_init(void);
void av_to_current(void);
void motor_move(void);
void motor_move_foc(void);
void current_meas(void);
void motor_current_cal(void);

//HALL State:
enum hall_states {S1=1, S2, S3, S4, S5, S6};
char hall_state = -1; // i.e. not set yet


void motor_set_duty(void);
void motor_set_duty2(void); //setting the PWM duty based on sin lookup table
int MOTOR_MODE = STOPPED;
volatile int which_half = 0;
int average = 0;
int average1 = 0;
int average2 = 0;
int average3 = 0;
int av[3];
int duty = 0;
int I_qui = 0;
float phaseindex = M_PI;
int dutyU = 0;
int dutyV = 0;
int dutyW = 0;
int dutycalpoint_low = 0;
int dutycalpoint_high = 0;
int I_mA = 0;
int err_max_current = 0;
int err_no_motor = 0;
int curr[3];
float av_curr = 0;
int wanted_encoder_count = 0;
float vel = 0.0;

//sine array values lookup table: LEN = 256
//volatile unsigned char sine_array_index = 0;

//unsigned char sine_array[256]=
//{133,139,144,149,155,160,165,170,175,180,185,189,194,198,203,207,211,214,218,221,225,228,231,233,
//236,238,241,243,244,246,248,249,250,251,252,253,254,254,254,255,255,255,255,255,255,255,254,254,
//254,254,253,253,252,252,252,252,251,251,251,251,250,250,250,250,250,250,250,251,251,251,251,252,
//252,252,252,253,253,254,254,254,254,255,255,255,255,255,255,255,254,254,254,253,252,251,250,249,
//248,246,244,243,241,238,236,233,231,228,225,221,218,214,211,207,203,198,194,189,185,180,175,170,
//165,160,155,149,144,139,133,128,123,117,112,107,101,96,91,86,81,76,71,67,62,58,53,49,45,42,38,
//35,31,28,25,23,20,18,15,13,12,10,8,7,6,5,4,3,2,2,2,1,1,1,1,1,1,1,2,2,2,2,3,3,4,4,4,4,5,5,5,5,6,
//6,6,6,6,6,6,5,5,5,5,4,4,4,4,3,3,2,2,2,2,1,1,1,1,1,1,1,2,2,2,3,4,5,6,7,8,10,12,13,15,18,20,23,25,
//28,31,35,38,42,45,49,53,58,62,67,71,76,81,86,91,96,101,107,112,117,123,128};

short int sine_array[1024] = {0,3,7,11,14,18,22,25,29,33,36,40,44,47,51,55,58,62,66,69,73,77,80,84,88,
		91,95,98,102,106,109,113,117,120,124,127,131,135,138,142,145,149,152,156,160,163,167,170,174,
		177,181,184,188,191,195,198,202,205,209,212,215,219,222,226,229,233,236,239,243,246,249,253,
		256,259,263,266,269,273,276,279,282,286,289,292,295,298,302,305,308,311,314,317,321,324,327,
		330,333,336,339,342,345,348,351,354,357,360,363,366,369,372,374,377,380,383,386,389,391,394,
		397,400,402,405,408,411,413,416,419,421,424,426,429,432,434,437,439,442,444,447,449,451,454,
		456,459,461,463,466,468,470,473,475,477,479,481,484,486,488,490,492,494,496,498,500,502,504,
		506,508,510,512,514,516,518,520,522,523,525,527,529,530,532,534,535,537,539,540,542,543,545,
		547,548,550,551,552,554,555,557,558,559,561,562,563,564,566,567,568,569,570,572,573,574,575,
		576,577,578,579,580,581,582,582,583,584,585,586,587,587,588,589,589,590,591,591,592,592,593,
		594,594,595,595,595,596,596,597,597,597,598,598,598,598,599,599,599,599,599,599,599,599,600,
		600,599,599,599,599,599,599,599,599,598,598,598,598,597,597,597,596,596,595,595,595,594,594,
		593,592,592,591,591,590,589,589,588,587,587,586,585,584,583,582,582,581,580,579,578,577,576,
		575,574,573,572,570,569,568,567,566,564,563,562,561,559,558,557,555,554,552,551,550,548,547,
		545,543,542,540,539,537,535,534,532,530,529,527,525,523,522,520,518,516,514,512,510,508,506,
		504,502,500,498,496,494,492,490,488,486,484,481,479,477,475,473,470,468,466,463,461,459,456,
		454,451,449,447,444,442,439,437,434,432,429,426,424,421,419,416,413,411,408,405,402,400,397,
		394,391,389,386,383,380,377,374,372,369,366,363,360,357,354,351,348,345,342,339,336,333,330,
		327,324,321,317,314,311,308,305,302,298,295,292,289,286,282,279,276,273,269,266,263,259,256,
		253,249,246,243,239,236,233,229,226,222,219,215,212,209,205,202,198,195,191,188,184,181,177,
		174,170,167,163,160,156,152,149,145,142,138,135,131,127,124,120,117,113,109,106,102,98,95,91,
		88,84,80,77,73,69,66,62,58,55,51,47,44,40,36,33,29,25,22,18,14,11,7,3,0,-4,-8,-12,-15,-19,-23,
		-26,-30,-34,-37,-41,-45,-48,-52,-56,-59,-63,-67,-70,-74,-78,-81,-85,-89,-92,-96,-99,-103,-107,
		-110,-114,-118,-121,-125,-128,-132,-136,-139,-143,-146,-150,-153,-157,-161,-164,-168,-171,-175,
		-178,-182,-185,-189,-192,-196,-199,-203,-206,-210,-213,-216,-220,-223,-227,-230,-234,-237,-240,
		-244,-247,-250,-254,-257,-260,-264,-267,-270,-274,-277,-280,-283,-287,-290,-293,-296,-299,-303,
		-306,-309,-312,-315,-318,-322,-325,-328,-331,-334,-337,-340,-343,-346,-349,-352,-355,-358,-361,
		-364,-367,-370,-373,-375,-378,-381,-384,-387,-390,-392,-395,-398,-401,-403,-406,-409,-412,-414,
		-417,-420,-422,-425,-427,-430,-433,-435,-438,-440,-443,-445,-448,-450,-452,-455,-457,-460,-462,
		-464,-467,-469,-471,-474,-476,-478,-480,-482,-485,-487,-489,-491,-493,-495,-497,-499,-501,-503,
		-505,-507,-509,-511,-513,-515,-517,-519,-521,-523,-524,-526,-528,-530,-531,-533,-535,-536,-538,
		-540,-541,-543,-544,-546,-548,-549,-551,-552,-553,-555,-556,-558,-559,-560,-562,-563,-564,-565,
		-567,-568,-569,-570,-571,-573,-574,-575,-576,-577,-578,-579,-580,-581,-582,-583,-583,-584,-585,
		-586,-587,-588,-588,-589,-590,-590,-591,-592,-592,-593,-593,-594,-595,-595,-596,-596,-596,-597,
		-597,-598,-598,-598,-599,-599,-599,-599,-600,-600,-600,-600,-600,-600,-600,-600,-600,-600,-600,
		-600,-600,-600,-600,-600,-600,-600,-599,-599,-599,-599,-598,-598,-598,-597,-597,-596,-596,-596,
		-595,-595,-594,-593,-593,-592,-592,-591,-590,-590,-589,-588,-588,-587,-586,-585,-584,-583,-583,
		-582,-581,-580,-579,-578,-577,-576,-575,-574,-573,-571,-570,-569,-568,-567,-565,-564,-563,-562,
		-560,-559,-558,-556,-555,-553,-552,-551,-549,-548,-546,-544,-543,-541,-540,-538,-536,-535,-533,
		-531,-530,-528,-526,-524,-523,-521,-519,-517,-515,-513,-511,-509,-507,-505,-503,-501,-499,-497,
		-495,-493,-491,-489,-487,-485,-482,-480,-478,-476,-474,-471,-469,-467,-464,-462,-460,-457,-455,
		-452,-450,-448,-445,-443,-440,-438,-435,-433,-430,-427,-425,-422,-420,-417,-414,-412,-409,-406,
		-403,-401,-398,-395,-392,-390,-387,-384,-381,-378,-375,-373,-370,-367,-364,-361,-358,-355,-352,
		-349,-346,-343,-340,-337,-334,-331,-328,-325,-322,-318,-315,-312,-309,-306,-303,-299,-296,-293,
		-290,-287,-283,-280,-277,-274,-270,-267,-264,-260,-257,-254,-250,-247,-244,-240,-237,-234,-230,
		-227,-223,-220,-216,-213,-210,-206,-203,-199,-196,-192,-189,-185,-182,-178,-175,-171,-168,-164,
		-161,-157,-153,-150,-146,-143,-139,-136,-132,-128,-125,-121,-118,-114,-110,-107,-103,-99,-96,-92,
		-89,-85,-81,-78,-74,-70,-67,-63,-59,-56,-52,-48,-45,-41,-37,-34,-30,-26,-23,-19,-15,-12,-8,-4,0
};

////for motor tracking to simulate engine rotation:
//int pos_counter = 0;


//int lookup_encoder_pos[] = {0, -100, -400, -500, -750, -1000};
//int LEN = 670;
//int cycles = 0; //for debugging, number of rotations to do before stopping
//float max_delta_phase = 0.024; //radians
//float delta_t = 1.49e-3; // [s] (1/671)
//float delta_angle = (2*M_PI)/3200;

//debugging variables:
//int arr_i[I_MAX];
//int arr_adc[I_MAX];
//int arr_adc1[I_MAX];
//int arr_adc2[I_MAX];
//int arr_adc3[I_MAX];
//int arr_duty[I_MAX];
//int arr_ImA[I_MAX];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int dummyphaseindex = 0;
int sine_idx = 0;
int phaseU = 0;
int phaseV = 0;
int phaseW = 0;

unsigned int encoder_time = 0;
unsigned int encoder_ticks = 0;
unsigned int encoder_rotations = 0;

volatile unsigned short cur1 = 0;
volatile unsigned short cur2 = 0;
volatile unsigned short cur3 = 0;

float v1, v2, v3;
float curr_fdbk1, curr_fdbk2, curr_fdbk3;
float tmp = 0;
float curr_alpha, curr_beta;
//float curr_alpha2, curr_beta2;
//float curr_alpha3, curr_beta3;
//float curr_alpha4, curr_beta4;
//float curr_alpha5, curr_beta5;
//float curr_alpha6, curr_beta6;
float curr_d, curr_q;
//float curr_d2, curr_q2;
//float curr_d3, curr_q3;
//float curr_d4, curr_q4;
//float curr_d5, curr_q5;
//float curr_d6, curr_q6;
float theta;
float Iu_QUIESCENT_mA, Iv_QUIESCENT_mA, Iw_QUIESCENT_mA;

//PI controller for Iq:
float sum_PI_Iq = 0;
float Iq_Kp = 0.1;//576;
float Iq_Ki = 0.01;
float Iq_Kc = 0.1;
float PI_Vq = 0;
float Iq_ref = 200;

//PI controller for Id:
float sum_PI_Id = 0;
float Id_Kp = 0.1;//576;
float Id_Ki = 0.01;
float Id_Kc = 0.1;
float PI_Vd = 0;
float Id_ref = 0;

//PI controller for speed:
float sum_PI_speed = 0;
float speed_Kp = 0.24;
float speed_Ki = 0.24;
float speed_Kc = 0.5;
float PI_speed = 0;
float speed_ref = 0.5; //Units?

//inverse transforms from Vq/Vd to Va/b/c:
float Va, Vb, Vc;

//debug:
float max_PI_Vd = 0;
float min_PI_Vd = 0;
float max_PI_Vq = 0;
float min_PI_Vq = 0;

float max_Va = 0;
float min_Va = 0;
float max_Vb = 0;
float min_Vb = 0;
float max_Vc = 0;
float min_Vc = 0;

//speed measurements:
int old_pos, new_pos;
float speed = 0;
idx = 0;


void current_meas(void) { //converting ADC samples into curr_fdbk measurements
	v1 = (float)cur1 / ADC_HALF_RANGE * VCC_DIV_2;
	curr_fdbk1 = (float)(v1 - AMP_V_BIAS) / AMP_GAIN * 1000 - Iu_QUIESCENT_mA;
//
	v2 = (float)cur2 / ADC_HALF_RANGE * VCC_DIV_2;
	curr_fdbk2 = (float)(v2 - AMP_V_BIAS) / AMP_GAIN * 1000 - Iv_QUIESCENT_mA;

	v3 = (float)cur3 / ADC_HALF_RANGE * VCC_DIV_2;
	curr_fdbk3 = (float)(v3 - AMP_V_BIAS) / AMP_GAIN * 1000 - Iw_QUIESCENT_mA;

	tmp = curr_fdbk1 + curr_fdbk2 + curr_fdbk3;

	//1 - AB
	curr_alpha = curr_fdbk1;
	curr_beta = 0.57735 * curr_fdbk1 + (0.57735 * curr_fdbk3 * 2);
//	//2 - AC
//	curr_alpha2 = curr_fdbk1;
//	curr_beta2 = 0.57735 * curr_fdbk1 + (0.57735 * curr_fdbk3 * 2);
//	//3 - BC
//	curr_alpha3 = curr_fdbk2;
//	curr_beta3 = 0.57735 * curr_fdbk2 + (0.57735 * curr_fdbk3 * 2);
//	//4 - BA
//	curr_alpha4 = curr_fdbk2;
//	curr_beta4 = 0.57735 * curr_fdbk2 + (0.57735 * curr_fdbk1 * 2);
//	//5 - CA
//	curr_alpha5 = curr_fdbk3;
//	curr_beta5 = 0.57735 * curr_fdbk3 + (0.57735 * curr_fdbk1 * 2);
//	//6 - CB
//	curr_alpha6 = curr_fdbk3;
//	curr_beta6 = 0.57735 * curr_fdbk3 + (0.57735 * curr_fdbk2 * 2);

	//theta = TIM2->CNT * ENC_TO_THETA;
	//normalise theta to between 0 and 2*pi
	theta = fmod((float)TIM2->CNT,(float)228.571)*0.0274889;

	//1
	curr_d = curr_alpha * cos(theta) + curr_beta * sin(theta);
	curr_q = -curr_alpha*sin(theta) + curr_beta*cos(theta);
//	//2
//	curr_d2 = curr_alpha2 * cos(theta) + curr_beta2 * sin(theta);
//	curr_q2 = -curr_alpha2*sin(theta) + curr_beta2*cos(theta);
//	//3
//	curr_d3 = curr_alpha3 * cos(theta) + curr_beta3 * sin(theta);
//	curr_q3 = -curr_alpha3*sin(theta) + curr_beta3*cos(theta);
//	//4
//	curr_d4 = curr_alpha4 * cos(theta) + curr_beta4 * sin(theta);
//	curr_q4 = -curr_alpha4*sin(theta) + curr_beta4*cos(theta);
//	//5
//	curr_d5 = curr_alpha5 * cos(theta) + curr_beta5 * sin(theta);
//	curr_q5 = -curr_alpha5*sin(theta) + curr_beta5*cos(theta);
//	//6
//	curr_d6 = curr_alpha6 * cos(theta) + curr_beta6 * sin(theta);
//	curr_q6 = -curr_alpha6*sin(theta) + curr_beta6*cos(theta);
//
//	if (fabs(curr_q) > 0.5) {
//		return;
//	}
//	else {
		if (MOTOR_MODE == INIT) {//find curr_q = 0, i.e. initialise the position of the motor
			TIM2->CNT = 0;
			Iu_QUIESCENT_mA = curr_fdbk1;
			Iv_QUIESCENT_mA = curr_fdbk2;
			Iw_QUIESCENT_mA = curr_fdbk3;
			new_pos = 0;
			old_pos = 0;
			MOTOR_MODE = TRACKING;
		}
//		return;
//	}
}

//void motor_move(void) {
//
//	int n = 64;
//	current_meas();
//
//	//this works but needs the sine array which is a full "bumpy" sine waveform
////	if (dummyphaseindex % n == 0) {
////		motor_set_duty2();
////		current_meas();
////		sine_array_index += 1;
////	}
//
//	motor_set_duty2();
//	sine_idx += n;
//
//	dummyphaseindex++;
//	if (dummyphaseindex > 15360*3) { //65535*3) {
//		motor_disable();
//		MOTOR_MODE = STOPPED;
//	}
//
//}

float PI_Controller_Iqref(void) {
	float err = Iq_ref - curr_q;//fmod(Iq_ref - curr_q,500); //need to slew rate limit this - add max rate of change
	float u = sum_PI_Iq + Iq_Kp * err;
	float outmax = 4000;// 12000;
	float outmin = -4000;//-12000;
	float out = 0;
	float excess = 0;

	if (u > outmax) {
		out = outmax;
	}
	else if (u < outmin) {
		out = outmin;
	}
	else {
		out = u;
	}
	excess = u - out;
	sum_PI_Iq += Iq_Ki*err - Iq_Kc*excess;
	return sum_PI_Iq;
}

float PI_Controller_Idref(void) {
	float err = Id_ref - curr_d; //need to slew rate limit this - add max rate of change
	float u = sum_PI_Id + Id_Kp * err;
	float outmax = 4000;//12000;
	float outmin = -4000;//-12000;
	float out = 0;
	float excess = 0;

	if (u > outmax) {
		out = outmax;
	}
	else if (u < outmin) {
		out = outmin;
	}
	else {
		out = u;
	}
	excess = u - out;
	sum_PI_Id += Id_Ki*err - Id_Kc*excess;
	return sum_PI_Id;
}

float PI_Controller_Speed_ref(void) {
	//float err = speed_ref - speed; //need to slew rate limit this - add max rate of change
	//float err = (speed_ref - TIM2->CNT);// fmod((speed_ref - TIM2->CNT),50);
	float err = (speed_ref - fmod((float)TIM2->CNT,(float)228.571)*0.0274889);
	float u = sum_PI_speed + speed_Kp * err;
	float outmax = 3.14;
	float outmin = -3.14;
	float out = 0;
	float excess = 0;

	if (u > outmax) {
		out = outmax;
	}
	else if (u < outmin) {
		out = outmin;
	}
	else {
		out = u;
	}
	excess = u - out;
	sum_PI_speed += speed_Ki*err - speed_Kc*excess;
	return sum_PI_speed;
}

void inverse_Clarke_Park(float Vq, float Vd) {
	float V_alpha = Vd * cos(theta) - Vq * sin(theta);
	float V_beta = Vd * sin(theta) + Vq * cos(theta);

	Va = V_alpha;
	Vb = (-V_alpha + sqrt(3)*V_beta)/2;
	Vc = (-V_alpha - sqrt(3)*V_beta)/2;
}

void motor_move_foc(void) {
	//int n = 10;
	current_meas();

	//if in "speed" control, then enable this block
//	if (dummyphaseindex % 240 == 0) {//this is the outer FOC loop for velocity calculations
//		//HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//		//velocity calc:
//		new_pos = TIM2->CNT;
//		speed = (float)(new_pos - old_pos);
//		speed *= 0.98175;//(TIM2->CCR2-TIM2->CCR1);
//		speed = TIM2->CNT;
//		old_pos = new_pos;
//		if (idx >= 0 && idx < 40) {
//			speed_ref += 10;
//			idx++;
//		}
//		else if (idx >= 40 && idx < 80) {
//			speed_ref -= 10;
//			idx++;
//		}
//		else {
//			idx = 0;
//		}
//	}
//	PI_speed = PI_Controller_Speed_ref();
//	Iq_ref = PI_speed;
	PI_Vq = PI_Controller_Iqref();

//	if (PI_Vq > max_PI_Vq) {
//		max_PI_Vq = PI_Vq;
//	}
//	else if (PI_Vq < min_PI_Vq) {
//		min_PI_Vq = PI_Vq;
//	}
//
	PI_Vd = PI_Controller_Idref();
//	if (PI_Vd > max_PI_Vd) {
//		max_PI_Vd = PI_Vd;
//	}
//	else if (PI_Vd < min_PI_Vd) {
//		min_PI_Vd = PI_Vd;
//	}


	inverse_Clarke_Park(PI_Vq,PI_Vd); //sets values for: Va/Vb/Vc
//	//a phase
//	if (Va > max_Va){
//		max_Va = Va;
//	}
//	else if (Va < min_Va) {
//		min_Va = Va;
//	}
//	//b phase
//	if (Vb > max_Vb){
//		max_Vb = Vb;
//	}
//	else if (Vb < min_Vb) {
//		min_Vb = Vb;
//	}
//	//c phase
//	if (Vc > max_Vc){
//		max_Vc = Vc;
//	}
//	else if (Vc < min_Vc) {
//		min_Vc = Vc;
//	}

	//convert voltages Va/b/c to PWM phases:
	Va += 16384;
	Va *= 0.03662;

	Vb += 16384;
	Vb *= 0.03662;

	Vc += 16384;
	Vc *= 0.03662;

//	if (TIM2->CNT == 3199) { //need to debounce this - get too many rotations
//	   encoder_time += HAL_GetTick();
//	   encoder_ticks = TIM2->CNT;
//	   encoder_rotations++;
//	}

	motor_set_duty3();
//	motor_set_duty2();
//	sine_idx += n;

	dummyphaseindex++;
	if (dummyphaseindex > 49152*6) { //65535*3) {
		motor_disable();
		MOTOR_MODE = STOPPED;
	}
}

void motor_set_duty3(void)
{
	dutyU = (int) Va;
	dutyV = (int) Vb;
	dutyW = (int) Vc;

	TIM1->CCR1 = dutyU;
	TIM1->CCR2 = dutyV;
	TIM1->CCR3 = dutyW;
}

void motor_set_duty2(void)
{
//	dutyU = (USER_TIM_PERIOD/2) + 1*(phaseU - 128);
//	dutyV = (USER_TIM_PERIOD/2) + 1*(phaseV - 128);
//	dutyW = (USER_TIM_PERIOD/2) + 1*(phaseW - 128);
//	unsigned char A = (sine_idx + DEGREE_120)%255;
//	unsigned char B = sine_idx%255;
//	unsigned char C = (sine_idx - DEGREE_120)%255;

	unsigned short int A = ((sine_idx + DEGREE_120_HIRES) >> 6);//8;
	unsigned short int B = ((sine_idx) >> 6);//8;
	unsigned short int C = ((sine_idx - DEGREE_120_HIRES) >> 6);//8;

	int idxA = A % 1024;
	int idxB = B % 1024;
	int idxC = C % 1024;
//	dutyU = (USER_TIM_PERIOD/2) + 1.0*(sine_array[A]-128); //sine_array from -128 to 128
//	dutyV = (USER_TIM_PERIOD/2) + 1.0*(sine_array[B]-128);
//	dutyW = (USER_TIM_PERIOD/2) + 1.0*(sine_array[C]-128);

	//new sine array
	dutyU = (USER_TIM_PERIOD/2) + 0.25*(sine_array[idxA]); //sine_array from -600 to 600
	dutyV = (USER_TIM_PERIOD/2) + 0.25*(sine_array[idxB]);
	dutyW = (USER_TIM_PERIOD/2) + 0.25*(sine_array[idxC]);

	//Write motor phases:
	TIM1->CCR1 = dutyU;
	TIM1->CCR2 = dutyV;
	TIM1->CCR3 = dutyW;
}

void motor_current_cal(void) {
	dutyU = (USER_TIM_PERIOD/2);
	dutyV = (USER_TIM_PERIOD/2);
	dutyW = (USER_TIM_PERIOD/2);

	//Write motor phases:
	TIM1->CCR1 = dutyU;
	TIM1->CCR2 = dutyV;
	TIM1->CCR3 = dutyW;

	current_meas();
}

void motor_enable(void) {

	HAL_GPIO_WritePin(PWM_EN_U_GPIO_Port, PWM_EN_U_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM_EN_V_GPIO_Port, PWM_EN_V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM_EN_W_GPIO_Port, PWM_EN_W_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(DIAG_EN_GPIO_Port, DIAG_EN_Pin, GPIO_PIN_SET);
}

void motor_disable(void) {
	dutyU = (USER_TIM_PERIOD/2);
	dutyV = (USER_TIM_PERIOD/2);
	dutyW = (USER_TIM_PERIOD/2);

	//Write motor phases:
	TIM1->CCR1 = dutyU;
	TIM1->CCR2 = dutyV;
	TIM1->CCR3 = dutyW;

	HAL_GPIO_WritePin(PWM_EN_U_GPIO_Port, PWM_EN_U_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWM_EN_V_GPIO_Port, PWM_EN_V_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWM_EN_W_GPIO_Port, PWM_EN_W_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(DIAG_EN_GPIO_Port, DIAG_EN_Pin, GPIO_PIN_RESET);
}

void motor_error(void) {
	duty = 0;
	//motor_set_duty();

	HAL_GPIO_WritePin(PWM_EN_U_GPIO_Port, PWM_EN_U_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWM_EN_V_GPIO_Port, PWM_EN_V_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PWM_EN_W_GPIO_Port, PWM_EN_W_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(DIAG_EN_GPIO_Port, DIAG_EN_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

 // HAL_TIM_Base_Start_IT(&htim2); //this is so that the HAL_TIM_PeriodElapsedCallback event fires (otherwise it doesn't)
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //timer2 tracks encoder ticks and stores the values in TIM2->CNT (values will range from 0 to 3200, and either inc/dec depending on direction of rotation)

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//  int cal_value = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  HAL_TIM_Base_Start_IT(&htim1);
//  //outer FOC loop interupt trigger
//  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //run motor for about 10s
//  TorqueCal();

  MOTOR_MODE = INIT;
  motor_enable();
  motor_current_cal();
 // current_meas();
//  MOTOR_MODE = STOPPED;
//  motor_disable();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  htim1.Init.Period = USER_TIM_PERIOD;
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1199;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
  sBreakDeadTimeConfig.Break2Filter = 7;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 7;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 7;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB2_LED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWM_EN_U_Pin|PWM_EN_V_Pin|PWM_EN_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_LED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = PB2_LED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_Yellow_Pin HALL_Green_Pin HALL_Blue_Pin */
  GPIO_InitStruct.Pin = HALL_Yellow_Pin|HALL_Green_Pin|HALL_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM_EN_U_Pin PWM_EN_V_Pin PWM_EN_W_Pin */
  GPIO_InitStruct.Pin = PWM_EN_U_Pin|PWM_EN_V_Pin|PWM_EN_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
/**
  * @brief NVIC Configuration.
  * @retval None
  */
//static void MX_NVIC_Init(void)
//{
//  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
//  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
//  /* TIM1_UP_TIM16_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
//  /* TIM1_TRG_COM_TIM17_IRQHandler */
//  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
//  /* ADC1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
//  HAL_NVIC_EnableIRQ(ADC1_IRQn);
//  /* TIM2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(TIM2_IRQn);
//  /* USART2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
//  HAL_NVIC_EnableIRQ(USART2_IRQn);
//  /* EXTI15_10_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//}

//void HAL_TIM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
//	if (htim->Instance == htim17.Instance) {// && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 ){
//		HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//	}
//	else {
//		//WTF
//		//call error handler
//	}
//}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
	cur1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	cur2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	cur3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
	current_meas();
//	HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//   if (htim->Instance == htim2.Instance) { //encoder should have tracked 3200 increments
//	   encoder_time = HAL_GetTick();
//	   encoder_ticks = TIM2->CNT;
//	   encoder_rotations += 1;
//	}
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_5) {
//		HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//	}
//	if (htim->Instance == htim6.Instance) {
//		HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//	}
	if (htim->Instance == htim1.Instance) { //i.e. the interrupt is coming from TIM1 (30kHz) then...
		if (MOTOR_MODE == TRACKING) {
			//motor_move();
			motor_move_foc();
		}
	}

}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	if (htim == htim1) {
//		HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//	}
//	else {
//	//HAL_GPIO_TogglePin(PB2_LED_GPIO_Port, PB2_LED_Pin);
//	if (fabs(encoder_count) >= fabs(lookup_encoder_pos[LEN])) {
//		MOTOR_MODE = STOPPED;
//		motor_disable();
//		return;
//	}
//    if (MOTOR_MODE == DOING) {
//    	track_to_encoder_position_list();
//    }
////    else {
////    	motor_disable();
////    }
//	}
//}

//logic see here --> http://yuandenghub.com/wp-content/uploads/2018/07/NI-Encoder-Tutorial-7109-en.pdf
//leaving this code commented out here to demonstrate how to identify which GPIO pin an interrupt was triggered from for future reference.
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == PB3_EncB_Pin) { //PB3 - EncB
//		encB_stuff();
//	}
//	else if (GPIO_Pin == PA15_EncA_Pin) { //PA15 - EncA
//		encA_stuff();
//	}
//}

////implementing a GPIO_EXTI callback function to service HALL sensor triggers on pins: PB11, PB12, PB14
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //triggers on a rising/falling edge of one of three hall sensor pins
//	//see here for encoding from hall sensor values to S1-S6: https://www.digikey.com.au/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
//	char y = -1;
//	char b = -1;
//	char g = -1;
//
//	y = HAL_GPIO_ReadPin(HALL_Yellow_GPIO_Port, HALL_Yellow_Pin);
//	b = HAL_GPIO_ReadPin(HALL_Blue_GPIO_Port, HALL_Blue_Pin);
//	g = HAL_GPIO_ReadPin(HALL_Green_GPIO_Port, HALL_Green_Pin);
//
//	if (b == GPIO_PIN_RESET && y == GPIO_PIN_RESET && g == GPIO_PIN_SET) {
//		hall_state = S1;
//		if (MOTOR_MODE == INIT) {
//			TIM2->CNT = 0;
//			MOTOR_MODE = TRACKING;
//		}
//	}
//	else if (b == GPIO_PIN_RESET && y == GPIO_PIN_SET && g == GPIO_PIN_SET) {
//		hall_state = S2;
//	}
//	else if (b == GPIO_PIN_RESET && y == GPIO_PIN_SET && g == GPIO_PIN_RESET) {
//		hall_state = S3;
//	}
//	else if (b == GPIO_PIN_SET && y == GPIO_PIN_SET && g == GPIO_PIN_RESET) {
//		hall_state = S4;
//	}
//	else if (b == GPIO_PIN_SET && y == GPIO_PIN_RESET && g == GPIO_PIN_RESET) {
//		hall_state = S5;
//	}
//	else if (b == GPIO_PIN_SET && y == GPIO_PIN_RESET && g == GPIO_PIN_SET) {
//		hall_state = S6;
//	}
//	else {
//		hall_state = -1;
//		//WTF! put error handler here
//	}
//}

//void DMATransferComplete(DMA_HandleTypeDef* hdma) {
//	//Disable UART DMA mode
//	huart2.Instance->CR3 &= ~USART_CR3_DMAT;
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
