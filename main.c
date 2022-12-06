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
//	__set_FAULTMASK(1);//�ر������ж�
	//0x8020000  //0xE0000
	//Task Create
    xTaskCreate(start_task,            	//Task Function
                "start_task",          	//��������
                START_STK_SIZE,        	//�����ջ��С
                NULL,                  	//���ݸ��������Ĳ���
                START_TASK_PRIO,       	//�������ȼ�
                NULL);   								//������              
    
	
	/* OS Resource Create*/
	AppTaskObject();

	vTaskStartScheduler();          	//�����������
	
	//�����������ܿ�������Ϊ��������Ŀռ䲻������������ʧ��
	while(1)
	{
		
	}
	/***********************************************/
}
 
//��ʼ����������
void start_task(void *pvParameters)
{
/*********************������ʼ��**************************/
	BSP_Init();
/***********************CPUռ��������*****************
	  xTaskCreate(cpu_task,     
                "cpu_task",   
                TASK_STK_SIZE, 
                NULL,
                CPU_TASK_PRIO,
                NULL); 	
/***********************FI����************************/
    xTaskCreate(fi_task,     
                "fi",   
                TASK_STK_SIZE, 
                NULL,
                FI_TASK_PRIO,
                NULL);
/***********************FI_DETE����*******************
	  xTaskCreate(fi_detect_task,     
                "fi_detect_task",   
                TASK_STK_SIZE, 
                NULL,
                FI_DETE_TASK_PRIO,
                NULL); 	
/***********************AI����************************/
    xTaskCreate(ai_task,     
                "ai",   
                TASK_STK_SIZE, 
                NULL,
                AI_TASK_PRIO,
                NULL); 	
/***********************DI����************************
    xTaskCreate(di_task,     
                "di",   
                TASK_STK_SIZE, 
                NULL,
                DI_TASK_PRIO,
                NULL); 
/***********************RTC����************************/
    xTaskCreate(rtc_task,     
                "rtc",   
                TASK_STK_SIZE, 
                NULL,
                DI_TASK_PRIO,
                NULL); 
/***********************ETH����************************/
		xTaskCreate(eth_task,     
				"eth",   
				TASK_STK_SIZE, 
				NULL,
				DI_TASK_PRIO,
				NULL); 
/***********************���1������************************/
		xTaskCreate(motor_task,     
				"motor",   
			   TASK_STK_SIZE, 
			   NULL,
			   MOTOR_TASK_PRIO,
			   NULL); 
/***********************���2������************************/
		xTaskCreate(motor_another_task,     
				"motor_another",   
			   TASK_STK_SIZE, 
			   NULL,
			   MOTOR_ANOTHER_TASK_PRIO,
			   NULL); 
/***********************�������**************************/
		xTaskCreate(supervisory_task,     
					"supervisory",   
					 TASK_STK_SIZE, 
					 NULL,
					 SUPERVISORY_PRIO,
					 NULL); 		
/***********************��������***************************
		xTaskCreate(warning_task,     
					"warning",   
					 TASK_STK_SIZE, 
					 NULL,
					 SUPERVISORY_PRIO,
					 NULL); 
/***********************CAN��������ͨ�ţ�****************/
		xTaskCreate(can_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 CAN_TASK_PRIO,
					 NULL); 		
/***********************������������ͨ�ţ�****************
	xTaskCreate(led_run_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 LED_RUN_PRIO,
					 NULL); 	
/***********************����BOOT��Ϣ����****************/
  xTaskCreate(boot_task,     
					"can",   
					 TASK_STK_SIZE, 
					 NULL,
					 BOOT_PRIO,
					 NULL); 	
/*********************************************************/					 
	while(1)
	{
		  // IWDG_Init(IWDG_Prescaler_32 ,625);             /////��ӿ��Ź�500ms�Զ���λ
		    
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



