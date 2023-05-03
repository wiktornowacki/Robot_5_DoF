/* USER CODE BEGIN Header */

/**
	Program sterujacy praca robota 5DoF
	Wykorzystano fragmenty kodu biblioteki AccelStepper
	https://www.airspayce.com/mikem/arduino/AccelStepper/
	Kod zgodnie z licencja udostepniono na zasadach GPL-V3
	
	5DoF robot control program
	Code fragments of the AccelStepper library were used
	https://www.airspayce.com/mikem/arduino/AccelStepper/
	The code according to the license is made available under the terms of GPL-V3
*/

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#define COUNT_OF_MOTORS 5
#define DELAY_AFTER_MOVEMENT 500
#define MIKROKROK 16
#define OKRES_OS_5 125

enum directions {DIRECTION_CCW, DIRECTION_CW};

int32_t _currentPos[COUNT_OF_MOTORS];
int32_t _targetPos[COUNT_OF_MOTORS];
float _speed[COUNT_OF_MOTORS];
float _maxSpeed[COUNT_OF_MOTORS];
float _acceleration[COUNT_OF_MOTORS];
float _sqrt_twoa[COUNT_OF_MOTORS];
int32_t _stepInterval[COUNT_OF_MOTORS];
int32_t _minPulseWidth[COUNT_OF_MOTORS];
int32_t _lastStepTime[COUNT_OF_MOTORS];

int32_t _n[COUNT_OF_MOTORS];
float _c0[COUNT_OF_MOTORS];
float _cn[COUNT_OF_MOTORS];
float _cmin[COUNT_OF_MOTORS];
enum directions _direction[COUNT_OF_MOTORS];


char buf[100];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float max(float a, float b) {
	if (a>b) return a;
	else return b;
}

void moveTo(long absolute, int i)
{
    if (_targetPos[i] != absolute)
    {
    _targetPos[i] = absolute;
    computeNewSpeed(i);
    }
}

void move(long relative, int i)
{
    moveTo(_currentPos[i] + relative, i);
}

int8_t runSpeed(int i)
{
    if (!_stepInterval[i])
    return 0;

    uint32_t time = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t nextStepTime = _lastStepTime[i] + _stepInterval[i];

    if (   ((nextStepTime >= _lastStepTime[i]) && ((time >= nextStepTime) || (time < _lastStepTime[i]))) || ((nextStepTime < _lastStepTime[i]) && ((time >= nextStepTime) && (time < _lastStepTime[i]))))
    {
		if (_direction[i] == DIRECTION_CW)
		{
			_currentPos[i] += 1;
		}
		else
		{
			_currentPos[i] -= 1;
		}
    step1(_currentPos[i], i);

    _lastStepTime[i] = time;
    return 1;
    }

    else {
    return 0;
    }
}

void step1(long step, int i)
{
	switch(i) {
	case 0:
		if (_direction[i] == DIRECTION_CCW) HAL_GPIO_WritePin(DIR_0_GPIO_Port, DIR_0_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_0_GPIO_Port, DIR_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_0_GPIO_Port, STEP_0_Pin, GPIO_PIN_SET);
		break;
	case 1:
		if (_direction[i] == DIRECTION_CCW) HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_1_GPIO_Port, STEP_1_Pin, GPIO_PIN_SET);
		break;
	case 2:
		if (_direction[i] == DIRECTION_CCW) HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_2_GPIO_Port, STEP_2_Pin, GPIO_PIN_SET);
		break;
	case 3:
		if (_direction[i] == DIRECTION_CCW) HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_3_GPIO_Port, STEP_3_Pin, GPIO_PIN_SET);
		break;
	case 4:
		if (_direction[i] == DIRECTION_CCW) HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_SET);
		break;
	}

	uint32_t time2 = __HAL_TIM_GET_COUNTER(&htim2);
	while (__HAL_TIM_GET_COUNTER(&htim2)-time2<_minPulseWidth[i]);

	switch(i) {
	case 0:
		HAL_GPIO_WritePin(STEP_0_GPIO_Port, STEP_0_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(STEP_1_GPIO_Port, STEP_1_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(STEP_2_GPIO_Port, STEP_2_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(STEP_3_GPIO_Port, STEP_3_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_RESET);
		break;
	}
}

long distanceToGo(int i) { return _targetPos[i] - _currentPos[i]; }
long targetPosition(int i) { return _targetPos[i]; }
long currentPosition(int i) { return _currentPos[i]; }

void setCurrentPosition(long position, int i)
{
    _targetPos[i] = _currentPos[i] = position;
    _n[i] = 0;
    _stepInterval[i] = 0;
}

void computeNewSpeed(int i)
{
    long distanceTo = distanceToGo(i);

    long stepsToStop = (long)((_speed[i] * _speed[i]) / (2.0 * _acceleration[i]));

    if (distanceTo == 0 && stepsToStop <= 1)
    {
    _stepInterval[i] = 0;
    _speed[i] = 0.0;
    _n[i] = 0;
    return;
    }

    if (distanceTo > 0)
    {
    if (_n[i] > 0)
    {
        if ((stepsToStop >= distanceTo) || _direction[i] == DIRECTION_CCW)
        _n[i] = -stepsToStop;
    }
    else if (_n[i] < 0)
    {
        if ((stepsToStop < distanceTo) && _direction[i] == DIRECTION_CW)
        _n[i] = -_n[i];
    }
    }
    else if (distanceTo < 0)
    {
    if (_n[i] > 0)
    {
        if ((stepsToStop >= -distanceTo) || _direction[i] == DIRECTION_CW)
        _n[i] = -stepsToStop;
    }
    else if (_n[i] < 0)
    {
        if ((stepsToStop < -distanceTo) && _direction[i] == DIRECTION_CCW)
        _n[i] = -_n[i];
    }
    }

    if (_n[i] == 0)
    {
    _cn[i] = _c0[i];
    _direction[i] = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
    _cn[i] = _cn[i] - ((2.0 * _cn[i]) / ((4.0 * _n[i]) + 1));
    _cn[i] = max(_cn[i], _cmin[i]);
    }
    _n[i]++;
    _stepInterval[i] = _cn[i];
    _speed[i] = 1000000.0 / _cn[i];
    if (_direction[i] == DIRECTION_CCW)
    _speed[i] = -_speed[i];

}

bool run(int i)
{
    if (runSpeed(i)) {
      computeNewSpeed(i);
    }
    return _speed[i] != 0.0 || distanceToGo(i) != 0;
}

void setMaxSpeed(float speed, int i)
{
    if (_maxSpeed[i] != speed)
    {
    _maxSpeed[i] = speed;
    _cmin[i] = 1000000.0 / speed;
    if (_n[i] > 0)
    {
        _n[i] = (long)((_speed[i] * _speed[i]) / (2.0 * _acceleration[i]));
        computeNewSpeed(i);
    }
    }
}

void setAcceleration(float acceleration, int i)
{
    if (acceleration == 0.0)
    return;
    if (_acceleration[i] != acceleration)
    {
    _n[i] = _n[i] * (_acceleration[i] / acceleration);
    _c0[i] = 0.676 * sqrt(2.0 / acceleration) * 1000000.0;
    _acceleration[i] = acceleration;
    computeNewSpeed(i);
    }
}

void setSpeed(float speed, int i)
{
    if (speed == _speed[i])
        return;
    if (speed<-_maxSpeed[i]) speed=-_maxSpeed[i];
    else if (speed>_maxSpeed[i]) speed=_maxSpeed[i];
    if (speed == 0.0)
    _stepInterval[i] = 0;
    else
    {
    _stepInterval[i] = fabs(1000000.0 / speed);
    _direction[i] = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed[i] = speed;
}

float speed(int i)
{
    return _speed[i];
}

void setMinPulseWidth(unsigned int minWidth, int i)
{
    _minPulseWidth[i] = minWidth;
}

void runToPosition(int i)
{
    while (run(i));
}

bool runSpeedToPosition(int i)
{
    if (_targetPos[i] == _currentPos[i])
    return false;
    if (_targetPos[i] >_currentPos[i])
    _direction[i] = DIRECTION_CW;
    else
    _direction[i] = DIRECTION_CCW;
    return runSpeed(i);
}

void runToNewPosition(long position, int i)
{
    moveTo(position, i);
    runToPosition(i);
}

void stop(int i)
{
    if (_speed[i] != 0.0)
    {
    long stepsToStop = (long)((_speed[i] * _speed[i]) / (2.0 * _acceleration[i])) + 1;
    if (_speed[i] > 0)
        move(stepsToStop, i);
    else
        move(-stepsToStop, i);
    }
}

void moveAll(int32_t x0, int32_t x1, int32_t x2, int32_t x3, int32_t x4) {
	if (x1>750) {
		while(1) {
			LAMPA_A_OFF();
			HAL_Delay(1000);
			LAMPA_A_ON();
			HAL_Delay(1000);
		}
	}


	moveTo(x0*MIKROKROK,0);
	moveTo(x1*MIKROKROK,1);
	moveTo(x2*MIKROKROK,2);
	moveTo(x3*MIKROKROK,3);
	moveTo(x4*MIKROKROK,4);

	int32_t max_a;
	int32_t max_b;
	int32_t max_max;

	x0 = abs(distanceToGo(0));
	x1 = abs(distanceToGo(1));
	x2 = abs(distanceToGo(2));
	x3 = abs(distanceToGo(3));
	x4 = abs(distanceToGo(4));

	if (x1>x0) max_a=x1;
	else max_a=x0;

	if (x3>x2) max_b=x3;
	else max_b=x2;

	if (x4>max_a && x4>max_b) max_max=x4;
	else if (max_a>max_b) max_max=max_a;
	else max_max=max_b;

	float czas_trwania_najdluzszego_ruchu = ((float)max_max)/((float)(250*MIKROKROK));

	setMaxSpeed((int)(x0/czas_trwania_najdluzszego_ruchu), 0);
	setMaxSpeed((int)(x1/czas_trwania_najdluzszego_ruchu), 1);
	setMaxSpeed((int)(x2/czas_trwania_najdluzszego_ruchu), 2);
	setMaxSpeed((int)(x3/czas_trwania_najdluzszego_ruchu), 3);
	setMaxSpeed((int)(x4/czas_trwania_najdluzszego_ruchu), 4);

	while( abs(distanceToGo(0))>0 ||  abs(distanceToGo(1))>0 || abs(distanceToGo(2))>0 || abs(distanceToGo(3))>0 || abs(distanceToGo(4))>0) {
		run(0);
		run(1);
		run(2);
		run(3);
		run(4);
	}
	HAL_Delay(DELAY_AFTER_MOVEMENT);

	for (int i=0; i<COUNT_OF_MOTORS; i++) {
	setMaxSpeed(200*MIKROKROK, i);
	}
}

void LAMPA_A_ON() {
	HAL_GPIO_WritePin(LAMPA_A_GPIO_Port, LAMPA_A_Pin, GPIO_PIN_RESET);
}

void LAMPA_B_ON() {
	HAL_GPIO_WritePin(LAMPA_B_GPIO_Port, LAMPA_B_Pin, GPIO_PIN_RESET);
}

void LAMPA_A_OFF() {
	HAL_GPIO_WritePin(LAMPA_A_GPIO_Port, LAMPA_A_Pin, GPIO_PIN_SET);
}

void LAMPA_B_OFF() {
	HAL_GPIO_WritePin(LAMPA_B_GPIO_Port,  LAMPA_B_Pin, GPIO_PIN_SET);
}


void czekaj() {
	HAL_Delay(500);
	LAMPA_A_OFF();
	LAMPA_B_ON();
	HAL_Delay(500);
	LAMPA_A_ON();
	LAMPA_B_OFF();
	HAL_Delay(500);
	LAMPA_A_ON();
	LAMPA_B_ON();
	HAL_Delay(500);
	LAMPA_A_OFF();
	LAMPA_B_OFF();
	HAL_Delay(500);
	LAMPA_A_ON();
	LAMPA_B_ON();
}

void PNEUMATIC_WSUN() {
	HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,  PNEUMATIC_Pin, GPIO_PIN_SET);
}

void PNEUMATIC_WYSUN() {
	HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port,  PNEUMATIC_Pin, GPIO_PIN_RESET);
}

void MAGNES_ON() {
	HAL_GPIO_WritePin(MAGNES_GPIO_Port,  MAGNES_Pin, GPIO_PIN_RESET);
}

void MAGNES_OFF() {
	HAL_GPIO_WritePin(MAGNES_GPIO_Port,  MAGNES_Pin, GPIO_PIN_SET);
}

void BLUE_ON() {
	HAL_GPIO_WritePin(BLUE_GPIO_Port,  BLUE_Pin, GPIO_PIN_RESET);
	RED_OFF();
	GREEN_OFF();
}

void BLUE_OFF() {
	HAL_GPIO_WritePin(BLUE_GPIO_Port,  BLUE_Pin, GPIO_PIN_SET);
}

void RED_ON() {
	HAL_GPIO_WritePin(RED_GPIO_Port,  RED_Pin, GPIO_PIN_RESET);
	GREEN_OFF();
	BLUE_OFF();
}

void RED_OFF() {
	HAL_GPIO_WritePin(RED_GPIO_Port,  RED_Pin, GPIO_PIN_SET);
}

void GREEN_ON() {
	HAL_GPIO_WritePin(GREEN_GPIO_Port,  GREEN_Pin, GPIO_PIN_RESET);
	RED_OFF();
	BLUE_OFF();
}

void GREEN_OFF() {
	HAL_GPIO_WritePin(GREEN_GPIO_Port,  GREEN_Pin, GPIO_PIN_SET);
}

void BUZZER_OFF() {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,  BUZZER_Pin, GPIO_PIN_SET);
}

void BUZZER_ON() {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,  BUZZER_Pin, GPIO_PIN_RESET);
}

void WHITE() {
	HAL_GPIO_WritePin(RED_GPIO_Port,  RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GREEN_GPIO_Port,  GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLUE_GPIO_Port,  BLUE_Pin, GPIO_PIN_RESET);
}

const float center_fix_of_base = 1.5;
const float ramie_a = 25.0;
const float ramie_b = 23.5;
const float ramie_c = 26.0;

float A;
float B;
float C;
float theta;

float x;
float y;
float c;
const float pi = 3.14159265359;

float sq (float a) {
	return a*a;
}

void Ruch_IK(float X_IK, float Y_IK, float Z_IK, float A_IK, float B_IK) {
	float KAT_podstawy = (atan2(X_IK,Y_IK))*(180/pi);

	x=sqrt(sq(X_IK)+sq(Y_IK));
	x=x-center_fix_of_base;
	y=Z_IK+26;

	c = sqrt( sq(x) + sq(y) );
	B = (acos( (sq(ramie_b) - sq(ramie_a) - sq(c))/(-2*ramie_a*c) )) * (180/pi);
	C = (acos( (sq(c) - sq(ramie_b) - sq(ramie_a))/(-2*ramie_a*ramie_b) )) * (180/pi);
	theta = (asin( y / c )) * (180/pi);
	float Silnik_2_Angle = 90-(B + theta);
	float Silnik_3_Angle = 180-C;
	float Silnik_4_Angle = -(180-(360-(90+B+theta+C)));

	float krok_1=(KAT_podstawy*200*6)/360;
	float krok_2=(Silnik_2_Angle*200*5.17*6)/360;
	float krok_3=(Silnik_3_Angle*200*5.17*(90/15))/360;
	float krok_4=(Silnik_4_Angle*200*4)/360;

	moveAll(krok_1, krok_2, krok_3, krok_4, (int)B_IK);
}

void Tasmociag_dopoki_nie_zadziala_czujnik() {
	while(HAL_GPIO_ReadPin(INPUT_A5_GPIO_Port, INPUT_A5_Pin) == GPIO_PIN_RESET) {
			for (int i=0; i<2000; i++) {
				uint32_t t_conv = __HAL_TIM_GET_COUNTER(&htim2);
				while(__HAL_TIM_GET_COUNTER(&htim2)-t_conv<=(uint32_t)500){}
				HAL_GPIO_TogglePin(CONVEYOR_GPIO_Port,  CONVEYOR_Pin);
			}
		}
}

void zerowanie() {
	for (int i=0; i<5; i++) {
				setCurrentPosition(0, i);
		}

	setSpeed(-100*MIKROKROK, 1);
	while(HAL_GPIO_ReadPin(INPUT_A2_GPIO_Port, INPUT_A2_Pin) == GPIO_PIN_SET) {
			runSpeed(1);
	}
	setCurrentPosition(0, 1);

	setSpeed(-200*MIKROKROK, 2);
	while(HAL_GPIO_ReadPin(INPUT_A4_GPIO_Port, INPUT_A4_Pin) == GPIO_PIN_SET) {
				runSpeed(2);
	}
	setCurrentPosition(0, 2);

	setSpeed(100*MIKROKROK, 3);
	while(HAL_GPIO_ReadPin(INPUT_A1_GPIO_Port, INPUT_A1_Pin) == GPIO_PIN_SET) {
				runSpeed(3);
		}
	setCurrentPosition(0, 3);

	setSpeed(-100*MIKROKROK, 4);
	while(HAL_GPIO_ReadPin(INPUT_A3_GPIO_Port, INPUT_A3_Pin) == GPIO_PIN_SET) {
				runSpeed(4);
		}
	setCurrentPosition(0, 4);

	setSpeed(-100*MIKROKROK, 0);
	while(HAL_GPIO_ReadPin(INPUT_A0_GPIO_Port, INPUT_A0_Pin) == GPIO_PIN_SET) {
		runSpeed(0);
	}
	setCurrentPosition(0, 0);

	moveAll(70,180,190,-50,300);

	for (int i=0; i<5; i++) {
		setCurrentPosition(0, i);
	}

	HAL_Delay(DELAY_AFTER_MOVEMENT);
}

void alarm() {
	while(1) {
		LAMPA_A_OFF();
		LAMPA_B_ON();
		HAL_Delay(500);
		LAMPA_A_ON();
		LAMPA_B_OFF();
		HAL_Delay(500);
	}
}

int ZnakNaInt (char a) {
	int x = a - '0';
	return (x);
}

void Delay_US(uint32_t czas) {
	uint32_t time = __HAL_TIM_GET_COUNTER(&htim2);

	while (__HAL_TIM_GET_COUNTER(&htim2)-time<=czas) {
		continue;
	}
}

void Obrot_5_Os(int pos) {
	if (pos>0) HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DIR_4_GPIO_Port, DIR_4_Pin, GPIO_PIN_RESET);

	for (int i=0; i<abs(pos)*MIKROKROK; i++) {
		HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_SET);
		Delay_US(OKRES_OS_5);
		HAL_GPIO_WritePin(STEP_4_GPIO_Port, STEP_4_Pin, GPIO_PIN_RESET);
		Delay_US(OKRES_OS_5);
	}

	HAL_Delay(DELAY_AFTER_MOVEMENT);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  RetargetInit(&huart2);

  for (int i=0; i<COUNT_OF_MOTORS; i++) {
	  _currentPos[i] = 0;
	  _targetPos[i] = 0;
	  _speed[i] = 0.0;
	  _maxSpeed[i] = 1.0;
	  _acceleration[i] = 0.0;
	  _sqrt_twoa[i] = 1.0;
	  _stepInterval[i] = 0;
	  _minPulseWidth[i] = 1;
	  _lastStepTime[i] = 0;

	  _n[i] = 0;
	  _c0[i] = 0.0;
	  _cn[i] = 0.0;
	  _cmin[i] = 1.0;
	  _direction[i] = DIRECTION_CCW;
  }

	setMaxSpeed(250*MIKROKROK, 0);
	setAcceleration(500*MIKROKROK, 0);
	setMaxSpeed(250*MIKROKROK, 1);
	setAcceleration(500*MIKROKROK, 1);
	setMaxSpeed(250*MIKROKROK, 2);
	setAcceleration(500*MIKROKROK, 2);
	setMaxSpeed(250*MIKROKROK, 3);
	setAcceleration(500*MIKROKROK, 3);
	setMaxSpeed(250*MIKROKROK, 4);
	setAcceleration(500*MIKROKROK, 4);
	PNEUMATIC_WSUN();
	MAGNES_OFF();
	BLUE_OFF();
	RED_OFF();
	GREEN_OFF();
	BUZZER_ON();
	HAL_Delay(50);
	BUZZER_OFF();

	LAMPA_A_OFF();
	LAMPA_B_OFF();
	HAL_Delay(500);

	while(1) {
		scanf("%s", buf);
		if (buf[0]=='a') {
			Tasmociag_dopoki_nie_zadziala_czujnik();
			printf("a\n");}
		else if (buf[0]=='b') {
			zerowanie();
			printf("b\n");
		}
		else if (buf[0]=='c') {
			if (ZnakNaInt(buf[1])!=1) {
				printf("e\n");
				alarm();
			}
			else {
				int x_uart = ZnakNaInt(buf[2])*100+ZnakNaInt(buf[3])*10+ZnakNaInt(buf[4]);
				int y_uart = ZnakNaInt(buf[5])*100+ZnakNaInt(buf[6])*10+ZnakNaInt(buf[7]);
				int z_uart = ZnakNaInt(buf[8])*100+ZnakNaInt(buf[9])*10+ZnakNaInt(buf[10]);
				int znak = ZnakNaInt(buf[11])-1;
				if (znak==1 || znak==3 || znak==5 || znak==7) x_uart*=(-1);
				if (znak==2 || znak==3 || znak==6 || znak==7) y_uart*=(-1);
				if (znak>=4) z_uart*=(-1);
				float x_uart_f = (float)(x_uart)/10.0;
				float y_uart_f = (float)(y_uart)/10.0;
				float z_uart_f = (float)(z_uart)/10.0;
				if (y_uart<0) alarm();
				Ruch_IK(x_uart_f, y_uart_f, z_uart_f, 0, 0);
				printf("c\n");
			}
		}
		else if (buf[0]=='d') {
			PNEUMATIC_WSUN();
		}
		else if (buf[0]=='f') {
			PNEUMATIC_WYSUN();
		}
		else if (buf[0]=='g') {
			MAGNES_ON();
		}
		else if (buf[0]=='h') {
			MAGNES_OFF();
		}
		else if (buf[0]=='i') {
			LAMPA_A_OFF();
		}
		else if (buf[0]=='j') {
			LAMPA_A_ON();
		}
		else if (buf[0]=='k') {
			LAMPA_B_OFF();
		}
		else if (buf[0]=='m') {
			LAMPA_B_ON();
		}
		else if (buf[0]=='n') {
			long odczyt = 1000000;
			if (HAL_GPIO_ReadPin(INPUT_A0_GPIO_Port, INPUT_A0_Pin) == GPIO_PIN_SET) odczyt+=100000;
			if (HAL_GPIO_ReadPin(INPUT_A2_GPIO_Port, INPUT_A2_Pin) == GPIO_PIN_SET) odczyt+=10000;
			if (HAL_GPIO_ReadPin(INPUT_A4_GPIO_Port, INPUT_A4_Pin) == GPIO_PIN_SET) odczyt+=1000;
			if (HAL_GPIO_ReadPin(INPUT_A1_GPIO_Port, INPUT_A1_Pin) == GPIO_PIN_SET) odczyt+=100;
			if (HAL_GPIO_ReadPin(INPUT_A3_GPIO_Port, INPUT_A3_Pin) == GPIO_PIN_SET) odczyt+=10;
			if (HAL_GPIO_ReadPin(INPUT_A5_GPIO_Port, INPUT_A5_Pin) == GPIO_PIN_SET) odczyt+=1;
			printf("n%i\n", odczyt);
		}
		else if (buf[0]=='o') {
			Obrot_5_Os(550);
		}
		else if (buf[0]=='p') {
			Obrot_5_Os(-550);
		}
		else if (buf[0]=='q') {
			RED_ON();
		}
		else if (buf[0]=='r') {
			RED_OFF();
		}
		else if (buf[0]=='s') {
			BLUE_ON();
		}
		else if (buf[0]=='t') {
			BLUE_OFF();
		}
		else if (buf[0]=='u') {
			GREEN_ON();
		}
		else if (buf[0]=='v') {
			GREEN_OFF();
		}
		else if (buf[0]=='w') {
			WHITE();
		}
		else if (buf[0]=='y') {
					BUZZER_ON();
				}
		else if (buf[0]=='z') {
					BUZZER_OFF();
				}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOC, BUZZER_Pin|LAMPA_B_Pin|STEP_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_0_Pin|DIR_0_Pin|STEP_1_Pin|STEP_3_Pin
                          |DIR_2_Pin|LAMPA_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_Pin|BLUE_Pin|DIR_3_Pin|MAGNES_Pin
                          |PNEUMATIC_Pin|GREEN_Pin|CONVEYOR_Pin|STEP_4_Pin
                          |DIR_4_Pin|DIR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_A5_Pin INPUT_A4_Pin */
  GPIO_InitStruct.Pin = INPUT_A5_Pin|INPUT_A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LAMPA_B_Pin STEP_2_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LAMPA_B_Pin|STEP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_A0_Pin INPUT_A1_Pin INPUT_A2_Pin */
  GPIO_InitStruct.Pin = INPUT_A0_Pin|INPUT_A1_Pin|INPUT_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_0_Pin DIR_0_Pin STEP_1_Pin STEP_3_Pin
                           DIR_2_Pin LAMPA_A_Pin */
  GPIO_InitStruct.Pin = STEP_0_Pin|DIR_0_Pin|STEP_1_Pin|STEP_3_Pin
                          |DIR_2_Pin|LAMPA_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_A3_Pin */
  GPIO_InitStruct.Pin = INPUT_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin BLUE_Pin DIR_3_Pin MAGNES_Pin
                           PNEUMATIC_Pin GREEN_Pin CONVEYOR_Pin STEP_4_Pin
                           DIR_4_Pin DIR_1_Pin */
  GPIO_InitStruct.Pin = RED_Pin|BLUE_Pin|DIR_3_Pin|MAGNES_Pin
                          |PNEUMATIC_Pin|GREEN_Pin|CONVEYOR_Pin|STEP_4_Pin
                          |DIR_4_Pin|DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
