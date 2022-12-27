#include "include.h"

u8 deviation_roating_speed_flag = 0; //ת�ٲ��ʶ
u8 sub_speed_count = 0;				 //������ѹ��ת�ٵ�������
u8 ano_csfs_last = 6;				 //�ӵ�λ
u8 ano_csps_last = 5;				 //�ӵ�λ
u8 ano_angle_change_falg = 0;		 //�ӽǶȸı�
u16 rotation_speed_s = 0;			 //����ѹ��ת��
u8 sl_rotation_speed_mutex = 0, sub_rotation_speed_mutex = 0; //ת�ٱ�ʶ��ת�ٲ��ʶ
u8 ano_infro_valiad_po = 0;				//1����ҶƬ��ʼ��ת��־��12�ȣ�
u8 ano_infro_valiad_ne = 0;				//1����ҶƬ��ʼ��ת��־��12�ȣ�
u8 ano_infro_valiad_det_po = 0;			//1����ҶƬ��ʼ��ת��־��11.6�ȣ�
u8 ano_infro_valiad_det_ne = 0;			//1����ҶƬ��ʼ��ת��־��11.6�ȣ�
u8 ano_infro_valiad_ex_po = 0;			//4����ҶƬ��ʼ��ת��־
u8 ano_infro_valiad_ex_ne = 0;			//4����ҶƬ��ʼ��ת��־
u8 ano_infro_valiad_ex_det_po = 0;
u8 ano_infro_valiad_ex_det_ne = 0;
u8 ano_infro_valiad_ex_ex_det_po = 0;
u8 ano_infro_valiad_ex_ex_det_ne = 0;

u8 ano_infor_unvalid_po = 0;
u8 ano_infor_unvalid_ne = 0;
u8 ano_infor_unvalid_Ref_po = 0;
u8 ano_infor_unvalid_Ref_ne = 0;
u8 info_ano_infor_unvalid_Ref_po = 0;
u8 info_ano_infor_unvalid_Ref_ne = 0;

int16_t ANO_Press_Valiad;
int16_t Ano_Press_Act;
u8 ano_exc_ang_flag = 0;//��ҶƬ����������������־λ

static void initMOTORData(void)
{
	// PWM_Pulstep(500);
	while (p_gSysUserRegs->WorkState == INITMode)
	{
		vTaskDelay(1);
	}
}

/*******************************************************************************
* Function Name  : motor_task
* Description    : motor_task
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void motor_another_task(void *pvParameters)
{
	u16 ANOTHER_ANGLE_ACT = 0;		//��ҶƬ�Ƕ�
	u16 ANGLE_ACT = 0;				//��ҶƬ�Ƕ�
	int ANGLE_ERR = 0;				//1����ҶƬ��Ŀ��ǶȲ�
	int ANOTHER_ANGLE_ERR = 0;		//1����ҶƬ��Ŀ��ǶȲ�
	int ANOTHER_ANGLE_REF = 0;		//1����ҶƬ��12�ȽǶȲ�
	int ANOTHER_ANGLE_REF_DET = 0;	//1����ҶƬ��11.6�ȽǶȲ�
	int ANOTHER_ANGLE_REF_EX = 0;
	int ANOTHER_ANGLE_REF_EX_DET = 0;
	int ANOTHER_ANGLE_REF_EX_EX = 0;
	int REF_UNVALID = 0;
	u8 plusorminu = 0;
	//��������־λ��1��λ������С����������2��λ����΢��������3��λ����ֹͣ������4��λ���򣨷Ŵ�����
	u8 UPorDOWN = 0, MOTOR_FLAG = 0, MOTOR_FLAG_Slim = 0, MOTOR_FLAG_UNVAD = 0, MOTOR_FLAG_Ref = 0;
	u8 fir_class_flag = 0;	//������ȼ���־λ
	u8 gear1_adj = 0;		//1��λʱ����������־λ
	u8 gear2_adj = 0;		//2��λʱ����������־λ
	u8 gear3_adj = 0;		//3��λʱ����������־λ
	u8 gear4_adj = 0;		//4��λʱ����������־λ
	u8 ano_dead_zone_ne = 0;//3����ҶƬ�����ʼ��ת��־λ
	u8 ano_dead_zone_po = 0;//3����ҶƬ�����ʼ��ת��־λ
	u8 dead_zone_adj1 = 0;	//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢����
	u8 dead_zone_adj2 = 0;	//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢����
	while (1)
	{
		if (xSemaphoreTake(xSem_UserData, portMAX_DELAY) == pdTRUE)
		{
			rotation_speed_s = ((p_gSysUserRegs->FI_Val[1]) * 30); //ת��2
			ANGLE_ACT = (u16)(p_gSysUserRegs->AI_Value[7]);
			ANOTHER_ANGLE_ACT = (u16)(p_gSysUserRegs->AI_Value[8]);
			ANO_Press_Valiad = p_gSysUserRegs->PRESS_VALAID;
			Ano_Press_Act = ((p_gSysUserRegs->AI_Value[5]) - 1000);
			xSemaphoreGive(xSem_UserData);
		}
#if BIHUAN
		if (angle_adj_flag == 1 && ANOTHER_ANGLE_ACT != 0)
		{
			/********************************��λ�ж�***********************************************/
			if (((ANOTHER_ANGLE_ACT < 11500) || (ANOTHER_ANGLE_ACT > 19000)))	//��ҶƬ�Ƕȳ����޶��Ƕ�
			{
				//��4����λ����ȫ�����ó���Ч״̬
				MOTOR_FLAG = STATUS_Unvalid;		//�޷�����1��λ����
				MOTOR_FLAG_Slim = STATUS_Unvalid;	//�޷�����2��λ����
				MOTOR_FLAG_UNVAD = STATUS_Unvalid;	//�޷�����3��λ����
				MOTOR_FLAG_Ref = STATUS_Unvalid;	//�޷�����4��λ����

				if (ANOTHER_ANGLE_ACT < 11500 && ano_exc_ang_flag == 0)
				{
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, 0, 25000, 25000, 25000, 25000, 25000); // 20hz ��ҶƬ��11.5�ȷ������
					ano_exc_ang_flag = 1;
				}
				else if (ANOTHER_ANGLE_ACT > 19000 && ano_exc_ang_flag == 0)
				{
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 20hz ��ҶƬ��19�ȷ������
					ano_exc_ang_flag = 1;
				}

				fir_class_flag = 0;	//����Ӧ�κνǶȵ�������
			}
			else if (ANOTHER_ANGLE_ACT >= 11500 && ANOTHER_ANGLE_ACT <= 19000)//��ҶƬ�Ƕ�δ�����޶��Ƕ�
			{
				//��4����λ����ȫ�����ó���Ч״̬
				MOTOR_FLAG = STATUS_Valid;		//���Խ���1��λ����
				MOTOR_FLAG_Slim = STATUS_Valid;	//���Խ���2��λ����
				MOTOR_FLAG_UNVAD = STATUS_Valid;//���Խ���3��λ����
				MOTOR_FLAG_Ref = STATUS_Valid;	//���Խ���4��λ����

				if(1 == ano_exc_ang_flag)//���ӳ���������������֮��������޶�����������ͣת
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);
				}

				ano_exc_ang_flag = 0;	//��ҶƬ���ޱ�־λ��λ

				fir_class_flag = 1;
			}

			/*******************************��λ�ж�***********************************************/
			if(ANO_Press_Valiad <= 2200 && fir_class_flag == 1)		//�����뵽��С������ 1��
			{
				gear2_adj = STATUS_Unvalid;	//2��λ����λ������Ч
				gear3_adj = STATUS_Unvalid;	//3��λ����λ������Ч
				gear4_adj = STATUS_Unvalid;	//4��λ����λ������Ч

				ANGLE_ERR = ANGLE_ACT - 11600;						//������ҶƬ��Ŀ��ǶȲ�
				ANOTHER_ANGLE_ERR = ANOTHER_ANGLE_ACT - 11600;		//������ҶƬ��Ŀ��ǶȲ�
				if(abs(ANOTHER_ANGLE_ERR - ANGLE_ERR) > 400)		//������ҶƬ�ǶȲ����0.4�ȣ�Ҫ����Ŀ��Ƕ�11.6����Զ��ҶƬ������11.6��
				{
					if(ANOTHER_ANGLE_ERR < ANGLE_ERR)				//����ҶƬ��Ŀ��Ƕ�11.6�����
					{
						gear1_adj = 1;								//��ҶƬ1��λȥ�ܣ���ҶƬ1��λʹ�ܣ�
					}
					else if(ANOTHER_ANGLE_ERR >= ANGLE_ERR)			//����ҶƬ��Ŀ��Ƕ�11.6�����
					{
						gear1_adj = 0;								//��ҶƬ1��λʹ�ܣ���ҶƬ1��λȥ�ܣ�
					}
				}
				else if (abs(ANOTHER_ANGLE_ERR - ANGLE_ERR) <= 400)	//������ҶƬ�ǶȲ�С�ڵ���0.4��
				{
					gear1_adj = 0;									//��ҶƬ4��λʹ�ܣ���ҶƬ4��λʹ�ܣ�
				}
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && (fir_class_flag == 1))	//��Ӧ������΢���� 2��
			{
				gear1_adj = STATUS_Unvalid;	//1��λ����λ������Ч
				gear3_adj = STATUS_Unvalid;	//3��λ����λ������Ч
				gear4_adj = STATUS_Unvalid;	//4��λ����λ������Ч

				if (abs(ANGLE_ACT - ANOTHER_ANGLE_ACT) > 200 && ANOTHER_ANGLE_ACT > 12000)			//������ҶƬ�ǶȲ����0.2���Ҵ�ҶƬ�Ƕȴ���12��
				{
					if (ANOTHER_ANGLE_ACT > ANGLE_ACT && dead_zone_adj1 == 0)
					{
						gear2_adj = 1;	//��ҶƬ2��λʹ�ܣ���ҶƬ2��λȥ�ܣ�
					}
					else if (ANOTHER_ANGLE_ACT <= ANGLE_ACT)
					{
						gear2_adj = 0;	//��ҶƬ2��λʹȥ�ܣ���ҶƬ2��λʹ�ܣ�
						dead_zone_adj1 = 0;
						dead_zone_adj2 = 0;
					}
				}
				else if (abs(ANGLE_ACT - ANOTHER_ANGLE_ACT) < 200 && dead_zone_adj2 == 0 && ANOTHER_ANGLE_ACT > 12000)//������ҶƬ�ǶȲ�С��0.2���Ҵ�ҶƬ�Ƕȴ���12��
				{
					gear2_adj = 2;		//��ҶƬ���յ��ÿ10-20��/Ȧ������΢����С����ҶƬ���յ��ÿ10-20��/Ȧ������΢����С����
				}
				else if (ANOTHER_ANGLE_ACT < 12000)	//����ҶƬ�Ƕ�С��12�ȣ���ͣ
				{
					gear2_adj = 0;
					dead_zone_adj1 = 0;
					dead_zone_adj2 = 0;
				}
			}
			else if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && (fir_class_flag == 1) && (ANOTHER_ANGLE_ACT > 12000))//��Ӧ����ֹͣ�� 3��
			{
				gear1_adj = STATUS_Unvalid;	//1��λ����λ������Ч
				gear2_adj = STATUS_Unvalid;	//2��λ����λ������Ч
				gear4_adj = STATUS_Unvalid;	//4��λ����λ������Ч

				if (abs(ANOTHER_ANGLE_ACT - ANGLE_ACT) > 200)	//������ҶƬ�ǶȲ����200
				{
												//��ҶƬ3��λȥ�ܣ���ҶƬ4��λȥ�ܣ�
					if (ANOTHER_ANGLE_ACT > ANGLE_ACT && ano_dead_zone_po == 0) //����ҶƬ�Ƕȴ�����ҶƬ�Ƕ�
					{
						gear3_adj = 1;
					}
					else if (ANOTHER_ANGLE_ACT <= ANGLE_ACT)//����ҶƬ�Ƕ�С�ڵ�����ҶƬ�Ƕ�
					{
						gear3_adj = 0;
						ano_dead_zone_po = 0;	//�뿪�����䣬��������ѷ��ͱ�־��λ
					}
				}
				else if (abs(ANOTHER_ANGLE_ACT - ANGLE_ACT) < 200)//������ҶƬ�ǶȲ�С��200
				{
					gear3_adj = 0;
					ano_dead_zone_po = 0;
					ano_dead_zone_ne = 0;
				}
			}
			else if (ANO_Press_Valiad >= 2550 && fir_class_flag == 1)//��Ӧ���뿪���� 4��
			{
				gear1_adj = STATUS_Unvalid;	//1��λ����λ������Ч
				gear2_adj = STATUS_Unvalid;	//2��λ����λ������Ч
				gear3_adj = STATUS_Unvalid;	//3��λ����λ������Ч

				gear4_adj = 0;				//��ҶƬ4��λʹ�ܣ���ҶƬ4��λʹ�ܣ�
			}

			/*********************************��λִ��********************************************/
			/************������С������ 1��************/
			if (ANO_Press_Valiad <= 2200 && MOTOR_FLAG == 0 && gear1_adj == 0 && fir_class_flag == 1)//�����ϵ�ʱ��ҶƬ����1��
			{
				MOTOR_FLAG_Slim = STATUS_Valid;		//2��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;		//4��λ����λ

				dead_zone_adj1 = 0;					//2��ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ
				dead_zone_adj2 = 0;					//2��ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ

				ano_dead_zone_ne = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ
				ano_dead_zone_po = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ

				ano_infro_valiad_ex_ne = 0;			//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;			//4����ҶƬ��ʼ��ת��־��λ

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF = (ANOTHER_ANGLE_ACT - 12000);//����1��ʱ��ҶƬƫ��Ƕ�

				if (ANOTHER_ANGLE_REF < 0 && ano_infro_valiad_po == 0 && abs(ANOTHER_ANGLE_REF) > 200)//��ҶƬ�����һ������ & �Ƕ�С��12�� & �Ƕ�ƫ��0.2������
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_po = 1;
				}
				else if (ANOTHER_ANGLE_REF > 0 && ano_infro_valiad_ne == 0 && abs(ANOTHER_ANGLE_REF) > 200)//��ҶƬ�����һ������ & �Ƕȴ���12�� & �Ƕ�ƫ��0.2������
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ne = 1;
				}
				else if ((abs(ANOTHER_ANGLE_REF) < 200))//��ҶƬƫ��Ƕ���0.2��֮�� & ������ת��ת���� & ��ҶƬ�綯����ͣ
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					MOTOR_FLAG = 1;

					ano_infro_valiad_po = 0;	//��λ
					ano_infro_valiad_ne = 0;	//��λ
				}
			}
			else if (ANO_Press_Valiad <= 2200 && MOTOR_FLAG == 1 && gear1_adj == 0 && fir_class_flag == 1)//�����תʱ��ҶƬ����1��
			{
				MOTOR_FLAG_Slim = STATUS_Valid;		//2��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;		//4��λ����λ

				dead_zone_adj1 = 0;					//2��ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ
				dead_zone_adj2 = 0;					//2��ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ

				ano_dead_zone_ne = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ
				ano_dead_zone_po = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ

				ano_infro_valiad_ex_ne = 0;			//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;			//4����ҶƬ��ʼ��ת��־��λ

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF_DET = (ANOTHER_ANGLE_ACT - 11800); //����1��ʱ��ҶƬƫ��Ƕȣ���11.8���㣬����11.6������0.2��ԣ��
				if (ANOTHER_ANGLE_REF_DET < 0 && ano_infro_valiad_det_po == 0 && abs(ANOTHER_ANGLE_REF_DET) > 100)//��ҶƬ�Ƕ�С��11.8�� & �Ƕ�ƫ��0.1������
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 168 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 5hz
					ano_infro_valiad_det_po = 1;
				}
				else if (ANOTHER_ANGLE_REF_DET > 0 && ano_infro_valiad_det_ne == 0 && abs(ANOTHER_ANGLE_REF_DET) > 100)//��ҶƬ����1�� & �Ƕȴ���11.8�� & �Ƕ�ƫ��0.1������
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 168 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 5hz
					ano_infro_valiad_det_ne = 1;
				}
				else if ((abs(ANOTHER_ANGLE_REF_DET) < 100) && ((ano_infro_valiad_det_ne == 1) || (ano_infro_valiad_det_po == 1)))//��ҶƬƫ��Ƕ���0.1��֮�� & ������ת��ת���� & ��ҶƬ�綯����ͣ
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					ano_infro_valiad_det_ne = 0;	//��λ
					ano_infro_valiad_det_po = 0;	//��λ
				}
			}

			/************���뿪���� 4��***************/
			if (ANO_Press_Valiad >= 2550 && MOTOR_FLAG_Ref == 0 && gear4_adj == 0 && fir_class_flag == 1)
			{
				MOTOR_FLAG = STATUS_Valid;		//1��λ����λ
				MOTOR_FLAG_Slim = STATUS_Valid;	//2��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;//3��λ����λ

				ano_infro_valiad_po = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_ne = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_det_po = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ
				ano_infro_valiad_det_ne = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ

				dead_zone_adj1 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ
				dead_zone_adj2 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ

				ano_dead_zone_ne = 0;			//3����ҶƬ�����ʼ��ת��־λ��λ
				ano_dead_zone_po = 0;			//3����ҶƬ�����ʼ��ת��־λ��λ

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF_EX = (ANOTHER_ANGLE_ACT - 16500);//����4��ʱ��ҶƬƫ��Ƕ�
				if (ANOTHER_ANGLE_REF_EX < 0 && ano_infro_valiad_ex_po == 0 && abs(ANOTHER_ANGLE_REF_EX) > 200)//��ҶƬС��16.5�ȣ���ƫ��Ƕȴ���0.2��
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 21 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ex_po = 1;
				}
				else if (ANOTHER_ANGLE_REF_EX > 0 && ano_infro_valiad_ex_ne == 0 && abs(ANOTHER_ANGLE_REF_EX) > 200)//��ҶƬ����16.5�ȣ���ƫ��Ƕȴ���0.2��
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 21 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ex_ne = 1;
				}
				else if (abs(ANOTHER_ANGLE_REF_EX) < 200)//��ҶƬ���ڻ�С��16.5�ȣ���ƫ��Ƕ�С��0.2��
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					ano_infro_valiad_ex_ne = 0;
					ano_infro_valiad_ex_po = 0;
				}
			}

			/************����΢���� 2��***************/
			if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 1 && dead_zone_adj1 == 0 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))//����ҶƬ΢����С
			{
				MOTOR_FLAG = STATUS_Valid;			//1��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;		//4��λ����λ

				ano_infro_valiad_po = 0;			//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_ne = 0;			//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_det_po = 0;		//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ
				ano_infro_valiad_det_ne = 0;		//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ

				ano_dead_zone_ne = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ
				ano_dead_zone_po = 0;				//3����ҶƬ�����ʼ��ת��־λ��λ

				ano_infro_valiad_ex_ne = 0;			//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;			//4����ҶƬ��ʼ��ת��־��λ

				tim_another_flag = 0;

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				dead_zone_adj1 = 1;
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 2 && dead_zone_adj2 == 0 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))//��ҶƬ΢����С����ҶƬҲ΢����С��
			{
				MOTOR_FLAG = STATUS_Valid;			//1��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;		//4��λ����λ

				ano_infro_valiad_po = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_ne = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_det_po = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ
				ano_infro_valiad_det_ne = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ

				ano_dead_zone_ne = 0;			//3����ҶƬ�����ʼ��ת��־λ��λ
				ano_dead_zone_po = 0;			//3����ҶƬ�����ʼ��ת��־λ��λ

				ano_infro_valiad_ex_ne = 0;		//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;		//4����ҶƬ��ʼ��ת��־��λ

				tim_another_flag = 0;

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				dead_zone_adj2 = 1;
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 0 && fir_class_flag == 1)//��ҶƬͣת
			{
				MOTOR_FLAG = STATUS_Valid;			//1��λ����λ
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;		//4��λ����λ

				TIM_Another_Shutdown();
				PWM_CTRL_SL(1);

				dead_zone_adj1 = 0;					//��λ
				dead_zone_adj2 = 0;					//��λ
			}

			/************����ֹͣ�� 3��***************/
			if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && MOTOR_FLAG_UNVAD == 0 && gear3_adj == 1 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))
			{
				MOTOR_FLAG = STATUS_Valid; 		//1��λ����λ
				MOTOR_FLAG_Slim = STATUS_Valid;	//2��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;	//4��λ����λ

				ano_infro_valiad_po = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_ne = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_det_po = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ
				ano_infro_valiad_det_ne = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ

				dead_zone_adj1 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ
				dead_zone_adj2 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ

				ano_infro_valiad_ex_ne = 0;		//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;		//4����ҶƬ��ʼ��ת��־��λ

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				ano_dead_zone_po = 1;

			}
			else if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && MOTOR_FLAG_UNVAD == 0 && gear3_adj == 0 && fir_class_flag == 1)
			{
				MOTOR_FLAG = STATUS_Valid; 		//1��λ����λ
				MOTOR_FLAG_Slim = STATUS_Valid;	//2��λ����λ
				MOTOR_FLAG_Ref = STATUS_Valid;	//4��λ����λ

				ano_infro_valiad_po = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_ne = 0;		//1����ҶƬ��ʼ��ת��־��12�ȣ���λ
				ano_infro_valiad_det_po = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ
				ano_infro_valiad_det_ne = 0;	//1����ҶƬ��ʼ��ת��־��11.6�ȣ���λ

				dead_zone_adj1 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ
				dead_zone_adj2 = 0;				//2����ҶƬ���ʹ�ܱ�־λ������ҶƬ΢������λ

				ano_infro_valiad_ex_ne = 0;		//4����ҶƬ��ʼ��ת��־��λ
				ano_infro_valiad_ex_po = 0;		//4����ҶƬ��ʼ��ת��־��λ

				TIM_Another_Shutdown();
				PWM_CTRL_SL(1);

				ano_dead_zone_po = 0;			//��λ
				ano_dead_zone_ne = 0;			//��λ
			}
		}
		//*******************************����*********************************************//
#else
		if (1 == p_gSysUserRegs->CAN2_Value[4])
		{
			PWM_CTRL_SL(0);
			Motor_another_Step = p_gSysUserRegs->CAN2_Value[5];
			Motor_another_Step = Motor_another_Step << 8;
			Motor_another_Step = (Motor_another_Step | p_gSysUserRegs->CAN2_Value[6]);

			Motor_another_Stop = 0;
			plusorminu = 0;
			plusorminu = p_gSysUserRegs->CAN2_Value[7];
			p_gSysUserRegs->CAN2_Value[4] = 0;
			p_gSysUserRegs->CAN2_Value[5] = 0;
			p_gSysUserRegs->CAN2_Value[6] = 0;
			p_gSysUserRegs->CAN2_Value[7] = 0;
			tim_another_flag = 1;

			Motor_another_Start(50000 - 1, 168 - 1, plusorminu, 25000, 25000, 25000, 25000, 25000); // 5HZ        //50%ռ�ձ� 8V //��������˵Ӧ����tim_another_flag =1
		}
		else if (2 == p_gSysUserRegs->CAN2_Value[4])
		{
			Motor_another_Stop = 2; //���ֹͣ��־λ
			tim_another_flag = 0;
			TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_Trigger, DISABLE);
		}
		if (Motor_another_Stop == 2)
		{
			TIM_Another_Shutdown();
			PWM_CTRL_SL(1);
		}
#endif
		vTaskDelay(5);
	}
}
