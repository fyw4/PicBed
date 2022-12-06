/*
*********************************************************************************************************
*
*                                       	the MAIN file
*                                         STM32F407IGT6
*											FreeRTOS 9.0
*
* Filename      : main.c
* Version       : V1.2
* Programmer(s) : LyuXin
*********************************************************************************************************

*/
#include "include.h"
#include "main.h"

static void AppTaskObject (void);


__IO uint32_t 				localtime;							
SYS_USER_REGS				gSysUserRegs;		 													//User Data structure
__IO SYS_USER_REGS 	* const p_gSysUserRegs = &gSysUserRegs;		

//OS resource
xSemaphoreHandle xSem_UserData 	= NULL;					  //user data access locker
xQueueHandle xQue_FI 			  = NULL;								//message queue for the FI TASK
xQueueHandle xQue_DISK 			= NULL;							  //message queue for the DISK TASK
xQueueHandle xQue_RDNC 			= NULL;								//message queue for the RDNC TASK
xSemaphoreHandle xSem_AI		= NULL;								//signal semaphore for AI Task
xQueueHandle xQue_OutPut 		= NULL;								//message queue for the OutPut TASK
xQueueHandle xQue_CAN1 			= NULL;								//message queue for the CAN TASK
xQueueHandle xQue_CAN2			= NULL;								//message queue for the CAN TASK
xQueueHandle xQue_BOOT      = NULL;								//messsage queue for the highest prority Task
///*******************************************************************************
//* Function Name  : vApplicationTickHook
//* Description    : This function is OS's tick hook function
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void vApplicationTickHook( void )
//{
//	  localtime++;							//used to calculate local times
//	  APP_FI_CalculateISR();		//call FI calculate function every 10ms
//}
/*
*********************************************************************************************************
*	Function Name	:main
*	Description		:the main function
*	Input			:None
*	Return			:None
*********************************************************************************************************
*/
int main(void)
{ 
	/**********************************************/
	//SCB->VTOR = FLASH_BASE | 0x20000;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x20000);
//	__set_FAULTMASK(1);//关闭所有中断
	//0x8020000  //0xE0000
	//Task Create
    xTaskCreate(start_task,            	//Task Function
                "start_task",          	//任务名称
                START_STK_SIZE,        	//任务堆栈大小
                NULL,                  	//传递给任务函数的参数
                START_TASK_PRIO,       	//任务优先级
                NULL);   								//任务句柄              
    
	
	/* OS Resource Create*/
	AppTaskObject();

	vTaskStartScheduler();          	//开启任务调度
	
	//如果到达这里，很可能是因为创建任务的空间不够，创建任务失败
	while(1)
	{
		
	}
	/***********************************************/
}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
/*********************驱动初始化**************************/
	BSP_Init();
/***********************CPU占用率任务*****************
	  xTaskCreate(cpu_task,     
                "cpu_task",   
                TASK_STK_SIZE, 
                NULL,
                CPU_TASK_PRIO,
                NULL); 	
/***********************FI任务************************/
    xTaskCreate(fi_task,     
                "fi",   
                TASK_STK_SIZE, 
                NULL,
                FI_TASK_PRIO,
                NULL);
/***********************FI_DETE任务*******************
	  xTaskCreate(fi_detect_task,     
                "fi_detect_task",   
                TASK_STK_SIZE, 
                NULL,
                FI_DETE_TASK_PRIO,
                NULL); 	
/***********************AI任务************************/
    xTaskCreate(ai_task,     
                "ai",   
                TASK_STK_SIZE, 
                NULL,
                AI_TASK_PRIO,
                NULL); 	
/***********************DI任务************************
    xTaskCreate(di_task,     
                "di",   
                TASK_STK_SIZE, 
                NULL,
                DI_TASK_PRIO,
                NULL); 
/***********************RTC任务************************/
    xTaskCreate(rtc_task,     
                "rtc",   
                TASK_STK_SIZE, 
                NULL,
                DI_TASK_PRIO,
                NULL); 
/***********************ETH任务************************/
		xTaskCreate(eth_task,     
				"eth",   
				TASK_STK_SIZE, 
				NULL,
				DI_TASK_PRIO,
				NULL); 
/***********************电机1号任务************************/
		xTaskCreate(motor_task,     
				"motor",   
			   TASK_STK_SIZE, 
			   NULL,
			   MOTOR_TASK_PRIO,
			   NULL); 
/***********************电机2号任务************************/
		xTaskCreate(motor_another_task,     
				"motor_another",   
			   TASK_STK_SIZE, 
			   NULL,
			   MOTOR_ANOTHER_TASK_PRIO,
			   NULL); 
/***********************监控任务**************************/
		xTaskCreate(supervisory_task,     
					"supervisory",   
					 TASK_STK_SIZE, 
					 NULL,
					 SUPERVISORY_PRIO,
					 NULL); 		
/***********************报警任务***************************
		xTaskCreate(warning_task,     
					"warning",   
					 TASK_STK_SIZE, 
					 NULL,
					 SUPERVISORY_PRIO,
					 NULL); 
/***********************CAN任务（数据通信）****************/
		xTaskCreate(can_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 CAN_TASK_PRIO,
					 NULL); 		
/***********************灯显任务（数据通信）****************
	xTaskCreate(led_run_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 LED_RUN_PRIO,
					 NULL); 	
/***********************接收BOOT信息任务****************/
  xTaskCreate(boot_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 BOOT_PRIO,
					 NULL); 	
/*********************************************************/					 
	while(1)
	{
		  // IWDG_Init(IWDG_Prescaler_32 ,625);             /////添加看门狗500ms自动复位
		    
	}
}

static void AppTaskObject (void)
{
	vSemaphoreCreateBinary( xSem_UserData );				//global user data locker
	xQue_DISK	= xQueueCreate(16,sizeof(MSG_TYPE));		//message Queue for FI TASK
	xQue_FI 	= xQueueCreate(4,sizeof(APP_FI_DATA));		//message Queue for FI TASK
//	xQue_RDNC	= xQueueCreate(8,sizeof(MSG_TYPE));			//message Queue for RDNC TASK
	xSem_AI 	= xSemaphoreCreateCounting( 1, 0 );			//signal semaphore for AI TASK
//	xQue_OutPut	= xQueueCreate(2,sizeof(MSG_TYPE));			//message Queue for RDNC TASK
//	xQue_CAN1	= xQueueCreate(12,sizeof(CanRxMsg));		//message Queue for CAN TASK
//	xQue_CAN2	= xQueueCreate(12,sizeof(CanRxMsg));		//message Queue for CAN TASK
  xQue_BOOT = xQueueCreate(1,sizeof(uint8_t));
	if((xSem_UserData == NULL) ||(xQue_DISK == NULL) ||(xQue_FI == NULL)||(xSem_AI == NULL))
	{
		//set LESs to display errCode
		while(1);
	}
}



