/**
 * \file
 * \brief Main application file
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#include "Platform.h"
#include "Application.h"
#include "Event.h"
#include "LED.h"
#include "WAIT1.h"
#include "CS1.h"
#include "Trigger.h"
#include "KeyDebounce.h"
#include "CLS1.h"
#include "Turn.h"
#include "Drive.h"
#include "Distance.h"
  #include "VL6180X.h"
  #include "GI2C1.h"
  #include "TofPwr.h"
  #include "TofCE1.h"
  #include "TofCE2.h"
  #include "TofCE3.h"
  #include "TofCE4.h"
#include "Buzzer.h"

#include "KIN1.h"
#if PL_CONFIG_HAS_KEYS
  #include "Keys.h"
#endif
#if PL_CONFIG_HAS_SHELL
  #include "CLS1.h"
  #include "Shell.h"
  #include "RTT1.h"
#endif
#if PL_CONFIG_HAS_BUZZER
  #include "Buzzer.h"
#endif
#if PL_CONFIG_HAS_RTOS
  #include "FRTOS1.h"
  #include "RTOS.h"
#endif
#if PL_CONFIG_HAS_QUADRATURE
  #include "Q4CLeft.h"
  #include "Q4CRight.h"
#endif
#if PL_CONFIG_HAS_MOTOR
  #include "Motor.h"
#endif
#if PL_CONFIG_BOARD_IS_ROBO_V2
  #include "PORT_PDD.h"
#endif
#if PL_CONFIG_HAS_LINE_FOLLOW
  #include "LineFollow.h"
#endif
#if PL_CONFIG_HAS_LCD_MENU
  #include "LCD.h"
#endif
#if PL_CONFIG_HAS_SNAKE_GAME
  #include "Snake.h"
#endif
#if PL_CONFIG_HAS_REFLECTANCE
  #include "Reflectance.h"
#endif
#include "Sumo.h"

#if PL_CONFIG_HAS_EVENTS

static void BtnMsg(int btn, const char *msg) {
#if PL_CONFIG_HAS_SHELL
  #if PL_CONFIG_HAS_SHELL_QUEUE
    uint8_t buf[48];

    UTIL1_strcpy(buf, sizeof(buf), "Button ");
    UTIL1_strcat(buf, sizeof(buf), msg);
    UTIL1_strcat(buf, sizeof(buf), ": ");
    UTIL1_strcatNum32s(buf, sizeof(buf), btn);
    UTIL1_strcat(buf, sizeof(buf), "\r\n");
    SHELL_SendString(buf);
  #else
    CLS1_SendStr("Button pressed: ", CLS1_GetStdio()->stdOut);
    CLS1_SendStr(msg, CLS1_GetStdio()->stdOut);
    CLS1_SendStr(": ", CLS1_GetStdio()->stdOut);
    CLS1_SendNum32s(btn, CLS1_GetStdio()->stdOut);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdOut);
  #endif
#endif
}

void APP_EventHandler(EVNT_Handle event) {
  /*! \todo handle events */
  switch(event) {
  case EVNT_STARTUP:
    {
      int i;
      for (i=0;i<5;i++) {
        LED1_Neg();
        WAIT1_Waitms(50);
      }
      LED1_Off();
    }
  case EVNT_LED_HEARTBEAT:
    LED2_Neg();
    break;
#if PL_CONFIG_NOF_KEYS>=1
  case EVNT_SW1_PRESSED:
	  LED2_Neg();
    BtnMsg(1, "pressed");
    setStartSumo(1);
    BUZ_Beep(700,100);
    //LF_StartStopFollowing();
    //setStateToCalibrateByButton();

     break;
  case EVNT_SW1_LPRESSED:
	  LED2_Neg();
	  BtnMsg(1,"long pressed");
	  break;

#endif
    default:
      break;
   } /* switch */
}
#endif /* PL_CONFIG_HAS_EVENTS */

#if PL_CONFIG_HAS_MOTOR /* currently only used for robots */
static const KIN1_UID RoboIDs[] = {
  /* 0: L20, V2 */ {{0x00,0x0D,0x00,0x00,0x67,0xCD,0xB7,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 1: L21, V2 */ {{0x00,0x05,0x00,0x00,0x4E,0x45,0xB7,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 2: L4, V1  */ {{0x00,0x0B,0xFF,0xFF,0x4E,0x45,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x24}},
  /* 3: L23, V2 */ {{0x00,0x0A,0x00,0x00,0x67,0xCD,0xB8,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 4: L11, V2 */ {{0x00,0x19,0x00,0x00,0x67,0xCD,0xB9,0x11,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 5: L5, V2 */  {{0x00,0x38,0x00,0x00,0x67,0xCD,0xB5,0x41,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 6: L3, V1 */  {{0x00,0x33,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x0A}},
  /* 7: L1, V1 */  {{0x00,0x19,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x25}},
};

#endif

static void APP_AdoptToHardware(void) {
  KIN1_UID id;
  uint8_t res;

  res = KIN1_UIDGet(&id);
  if (res!=ERR_OK) {
    for(;;); /* error */
  }
#if PL_CONFIG_HAS_MOTOR
  if (KIN1_UIDSame(&id, &RoboIDs[0])) { /* L20 */
#if PL_CONFIG_HAS_QUADRATURE
    //(void)Q4CRight_SwapPins(TRUE);
#endif
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), FALSE); /* invert left motor */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), FALSE); /* invert left motor */
  } else if (KIN1_UIDSame(&id, &RoboIDs[1])) { /* V2 L21 */
    /* no change needed */
  } else if (KIN1_UIDSame(&id, &RoboIDs[2])) { /* V1 L4 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* revert left motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[3])) { /* L23 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
  } else if (KIN1_UIDSame(&id, &RoboIDs[4])) { /* L11 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[5])) { /* L5, V2 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert right motor */
    (void)Q4CRight_SwapPins(TRUE);
  } else if (KIN1_UIDSame(&id, &RoboIDs[6])) { /* L3, V1 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert right motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[7])) { /* L1, V1 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert right motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  }
#endif
#if PL_CONFIG_HAS_QUADRATURE && PL_CONFIG_BOARD_IS_ROBO_V2
  /* pull-ups for Quadrature Encoder Pins */
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
#endif
}

int startSumo = 0;
int globalStart = 0;

int getGlobalStart()
{
	return globalStart;
}

void setGlobalStart(int value)
{
	globalStart = value;
}

int getStartSumo()
{
	return startSumo;
}

void setStartSumo(int value)
{
	startSumo = value;
}




static void BlinkyTask(void *pvParameters)
{

	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{

		LED_Neg(1);
		vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);


	}
}


static void Startup(void *pvParameters)
{

	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{


		KEY_Scan();
		EVNT_HandleEvent(APP_EventHandler,1);
		vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);
	}
}


static void stayOnLine(void *pvParameters)
{

	TickType_t xLastWakeTime =  xTaskGetTickCount();

	for(;;)
	{
		if(getStartSumo() == 1){



			if(getGlobalStart() == 0)
			{
				WAIT1_Waitms(750);
			}

				setGlobalStart(1);


		if(REF_IsReady())
		{

			if(REF_GetLineKind() == REF_LINE_FULL)
			{
				if(DIST_GetDistance(DIST_SENSOR_FRONT) == -1){
				DRV_SetSpeed(6000,6000);
				}
				if(DIST_GetDistance(DIST_SENSOR_FRONT) < 350 && DIST_GetDistance(DIST_SENSOR_FRONT) > 0){
					BUZ_Beep(1000,100);
					DRV_SetSpeed(10000,10000);
					//My_BombBeep();
				}
				if(DIST_GetDistance(DIST_SENSOR_RIGHT) < 350 && DIST_GetDistance(DIST_SENSOR_RIGHT) > 0){
					//DRV_SetSpeed(10000,10000);
					TURN_Turn(TURN_RIGHT45,NULL);
					if(DIST_GetDistance(DIST_SENSOR_FRONT) < 350 && DIST_GetDistance(DIST_SENSOR_FRONT) > 0)
														{
															DRV_SetSpeed(10000,10000);

														}else
														{

															TURN_Turn(TURN_RIGHT45,NULL);

														}
					DRV_SetMode(DRV_MODE_SPEED);
				}
				if(DIST_GetDistance(DIST_SENSOR_LEFT) < 350 && DIST_GetDistance(DIST_SENSOR_LEFT) > 0){
									//DRV_SetSpeed(10000,10000);
									TURN_Turn(TURN_LEFT45,NULL);
									if(DIST_GetDistance(DIST_SENSOR_FRONT) < 350 && DIST_GetDistance(DIST_SENSOR_FRONT) > 0)
									{
										DRV_SetSpeed(10000,10000);

									}else
									{

										TURN_Turn(TURN_LEFT45,NULL);

									}

									DRV_SetMode(DRV_MODE_SPEED);

								}

				if(DIST_GetDistance(DIST_SENSOR_REAR) < 300 && DIST_GetDistance(DIST_SENSOR_REAR) > 0){
													TURN_Turn(TURN_LEFT180,NULL);
													DRV_SetMode(DRV_MODE_SPEED);
													//DRV_SetSpeed(-10000,-10000);
												}

			}
			else
			{

				//TURN_Turn(TURN_LEFT45,NULL);
				DRV_SetSpeed(-2000,2000);
				DRV_SetMode(DRV_MODE_SPEED);
				vTaskDelayUntil(&xLastWakeTime, 600/portTICK_PERIOD_MS);

			}
		}

		}

		vTaskDelayUntil(&xLastWakeTime, 5/portTICK_PERIOD_MS);
	}
}


static void goToStart(void *pvParameters)
{
	TickType_t xLastWakeTime =  xTaskGetTickCount();
	//DRV_SetMode(DRV_MODE_SPEED);
	for(;;)
	{/*
		Q4CLeft_QuadCntrType startPos = Q4CLeft_GetPos();
		//MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_LEFT),20);
		//MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT),20);
		DRV_SetSpeed(2000,2000);
		vTaskDelayUntil(&xLastWakeTime, 3000/portTICK_PERIOD_MS);

			DRV_SetSpeed(-2000,-2000);
			//MOT_SetSpeedPercent(MOT_GetMotorHandle(MOT_MOTOR_RIGHT),-20);
			vTaskDelayUntil(&xLastWakeTime, 3000/portTICK_PERIOD_MS);
			DRV_SetSpeed(0,0);
*/
			vTaskDelayUntil(&xLastWakeTime, 3000/portTICK_PERIOD_MS);


	}

}




void APP_Start(void) {
  PL_Init();
  APP_AdoptToHardware();

BaseType_t res;
xTaskHandle taskHndl;
res = xTaskCreate(BlinkyTask,"Blinky",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY+3,&taskHndl);
if(res != pdPASS)
{

}

BaseType_t res1;
xTaskHandle taskHndl1;
res1 = xTaskCreate(Startup,"Startup",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY+3,&taskHndl1);
if(res1 != pdPASS)
{

}


BaseType_t res2;
xTaskHandle taskHndl2;
res2 = xTaskCreate(stayOnLine,"stayOnLine",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY+4,&taskHndl2);


//BaseType_t res3;
//xTaskHandle taskHndl3;
//res3 = xTaskCreate(goToStart,"goToStart",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY,&taskHndl3);

vTaskStartScheduler();
 // TRG_SetTrigger (TRG_BOMB_BEEP, 5000/TRG_TICKS_MS , LED_HeartBeat , NULL) ;
  //TRG_SetTrigger(TRG_BOMB_BEEP,0,My_BombBeep,NULL);

  __asm volatile("cpsie i"); /* enable interrupts */
  for(;;) {

	  //BUZ_Beep(300000,10);
	    }
}


