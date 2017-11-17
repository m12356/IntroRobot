/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

#include "Platform.h" /* interface to the platform */
#if PL_CONFIG_HAS_SEMAPHORE
#include "FRTOS1.h"
#include "Sem.h"
#include "LED.h"

static xSemaphoreHandle sem = NULL;



static void vSlaveTask(void *pvParameters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
			for(;;)
			{


					 if(xSemaphoreTake(sem,5000)== pdTRUE)
					 {
					 LED1_Neg();


					 }



			}
}

static void vMasterTask(void *pvParameters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
			for(;;)
			{


					xSemaphoreGive(sem);

					vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);




			}
}

void SEM_Deinit(void) {
}

/*! \brief Initializes module */
void SEM_Init(void) {
	BaseType_t check1;
	xTaskHandle myMaster;
	BaseType_t check2;
	xTaskHandle mySlave;

	sem = xSemaphoreCreateBinary();




	check1 = xTaskCreate(vMasterTask,"Master",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY,&myMaster);
	if(check1 != pdPASS)
	{
		for(;;)
		{}

	}

	check2 = xTaskCreate(vSlaveTask,"Slave",configMINIMAL_STACK_SIZE+50,(void*)NULL,tskIDLE_PRIORITY,&mySlave);
		if(check2 != pdPASS)
		{
			for(;;){}
		}

}
#endif /* PL_CONFIG_HAS_SEMAPHORE */
