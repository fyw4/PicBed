#include "include.h"

u8 deviation_roating_speed_flag = 0; //转速差标识
u8 sub_speed_count = 0;				 //主从增压器转速调整次数
u8 ano_csfs_last = 6;				 //从挡位
u8 ano_csps_last = 5;				 //从挡位
u8 ano_angle_change_falg = 0;		 //从角度改变
u16 rotation_speed_s = 0;			 //从增压器转速
u8 sl_rotation_speed_mutex = 0, sub_rotation_speed_mutex = 0; //转速标识，转速差标识
u8 ano_infro_valiad_po = 0;				//1区从叶片开始正转标志（12度）
u8 ano_infro_valiad_ne = 0;				//1区从叶片开始反转标志（12度）
u8 ano_infro_valiad_det_po = 0;			//1区从叶片开始正转标志（11.6度）
u8 ano_infro_valiad_det_ne = 0;			//1区从叶片开始反转标志（11.6度）
u8 ano_infro_valiad_ex_po = 0;			//4区从叶片开始正转标志
u8 ano_infro_valiad_ex_ne = 0;			//4区从叶片开始反转标志
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
u8 ano_exc_ang_flag = 0;//从叶片超限情况电机启动标志位

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
	u16 ANOTHER_ANGLE_ACT = 0;		//从叶片角度
	u16 ANGLE_ACT = 0;				//主叶片角度
	int ANGLE_ERR = 0;				//1区从叶片与目标角度差
	int ANOTHER_ANGLE_ERR = 0;		//1区主叶片与目标角度差
	int ANOTHER_ANGLE_REF = 0;		//1区从叶片与12度角度差
	int ANOTHER_ANGLE_REF_DET = 0;	//1区主叶片与11.6度角度差
	int ANOTHER_ANGLE_REF_EX = 0;
	int ANOTHER_ANGLE_REF_EX_DET = 0;
	int ANOTHER_ANGLE_REF_EX_EX = 0;
	int REF_UNVALID = 0;
	u8 plusorminu = 0;
	//电机方向标志位，1挡位区域（最小开度区），2挡位区域（微调区），3挡位区域（停止区），4挡位区域（放大区）
	u8 UPorDOWN = 0, MOTOR_FLAG = 0, MOTOR_FLAG_Slim = 0, MOTOR_FLAG_UNVAD = 0, MOTOR_FLAG_Ref = 0;
	u8 fir_class_flag = 0;	//最高优先级标志位
	u8 gear1_adj = 0;		//1挡位时动作调整标志位
	u8 gear2_adj = 0;		//2挡位时动作调整标志位
	u8 gear3_adj = 0;		//3挡位时动作调整标志位
	u8 gear4_adj = 0;		//4挡位时动作调整标志位
	u8 ano_dead_zone_ne = 0;//3区从叶片电机开始正转标志位
	u8 ano_dead_zone_po = 0;//3区从叶片电机开始反转标志位
	u8 dead_zone_adj1 = 0;	//2区从叶片电机使能标志位（仅从叶片微调）
	u8 dead_zone_adj2 = 0;	//2区从叶片电机使能标志位（主从叶片微调）
	while (1)
	{
		if (xSemaphoreTake(xSem_UserData, portMAX_DELAY) == pdTRUE)
		{
			rotation_speed_s = ((p_gSysUserRegs->FI_Val[1]) * 30); //转速2
			ANGLE_ACT = (u16)(p_gSysUserRegs->AI_Value[7]);
			ANOTHER_ANGLE_ACT = (u16)(p_gSysUserRegs->AI_Value[8]);
			ANO_Press_Valiad = p_gSysUserRegs->PRESS_VALAID;
			Ano_Press_Act = ((p_gSysUserRegs->AI_Value[5]) - 1000);
			xSemaphoreGive(xSem_UserData);
		}
#if BIHUAN
		if (angle_adj_flag == 1 && ANOTHER_ANGLE_ACT != 0)
		{
			/********************************限位判断***********************************************/
			if (((ANOTHER_ANGLE_ACT < 11500) || (ANOTHER_ANGLE_ACT > 19000)))	//若叶片角度超过限定角度
			{
				//将4个挡位区域全部设置成无效状态
				MOTOR_FLAG = STATUS_Unvalid;		//无法进入1挡位区域
				MOTOR_FLAG_Slim = STATUS_Unvalid;	//无法进入2挡位区域
				MOTOR_FLAG_UNVAD = STATUS_Unvalid;	//无法进入3挡位区域
				MOTOR_FLAG_Ref = STATUS_Unvalid;	//无法进入4挡位区域

				if (ANOTHER_ANGLE_ACT < 11500 && ano_exc_ang_flag == 0)
				{
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, 0, 25000, 25000, 25000, 25000, 25000); // 20hz 从叶片向11.5度方向调节
					ano_exc_ang_flag = 1;
				}
				else if (ANOTHER_ANGLE_ACT > 19000 && ano_exc_ang_flag == 0)
				{
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 20hz 从叶片向19度方向调节
					ano_exc_ang_flag = 1;
				}

				fir_class_flag = 0;	//不响应任何角度调节请求
			}
			else if (ANOTHER_ANGLE_ACT >= 11500 && ANOTHER_ANGLE_ACT <= 19000)//若叶片角度未超过限定角度
			{
				//将4个挡位区域全部设置成有效状态
				MOTOR_FLAG = STATUS_Valid;		//可以进入1挡位区域
				MOTOR_FLAG_Slim = STATUS_Valid;	//可以进入2挡位区域
				MOTOR_FLAG_UNVAD = STATUS_Valid;//可以进入3挡位区域
				MOTOR_FLAG_Ref = STATUS_Valid;	//可以进入4挡位区域

				if(1 == ano_exc_ang_flag)//若从超限情况经电机调节之后进入了限定情况，将电机停转
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);
				}

				ano_exc_ang_flag = 0;	//从叶片超限标志位复位

				fir_class_flag = 1;
			}

			/*******************************挡位判断***********************************************/
			if(ANO_Press_Valiad <= 2200 && fir_class_flag == 1)		//若进入到最小开度区 1区
			{
				gear2_adj = STATUS_Unvalid;	//2挡位区域挡位调整无效
				gear3_adj = STATUS_Unvalid;	//3挡位区域挡位调整无效
				gear4_adj = STATUS_Unvalid;	//4挡位区域挡位调整无效

				ANGLE_ERR = ANGLE_ACT - 11600;						//主喷嘴叶片与目标角度差
				ANOTHER_ANGLE_ERR = ANOTHER_ANGLE_ACT - 11600;		//从喷嘴叶片与目标角度差
				if(abs(ANOTHER_ANGLE_ERR - ANGLE_ERR) > 400)		//若主从叶片角度差大于0.4度，要求离目标角度11.6度最远的叶片调整至11.6度
				{
					if(ANOTHER_ANGLE_ERR < ANGLE_ERR)				//若主叶片离目标角度11.6度最大
					{
						gear1_adj = 1;								//从叶片1挡位去能（主叶片1挡位使能）
					}
					else if(ANOTHER_ANGLE_ERR >= ANGLE_ERR)			//若从叶片离目标角度11.6度最大
					{
						gear1_adj = 0;								//从叶片1挡位使能（主叶片1挡位去能）
					}
				}
				else if (abs(ANOTHER_ANGLE_ERR - ANGLE_ERR) <= 400)	//若主从叶片角度差小于等于0.4度
				{
					gear1_adj = 0;									//从叶片4挡位使能（主叶片4挡位使能）
				}
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && (fir_class_flag == 1))	//若应进入如微调区 2区
			{
				gear1_adj = STATUS_Unvalid;	//1挡位区域挡位调整无效
				gear3_adj = STATUS_Unvalid;	//3挡位区域挡位调整无效
				gear4_adj = STATUS_Unvalid;	//4挡位区域挡位调整无效

				if (abs(ANGLE_ACT - ANOTHER_ANGLE_ACT) > 200 && ANOTHER_ANGLE_ACT > 12000)			//若主从叶片角度差大于0.2度且从叶片角度大于12度
				{
					if (ANOTHER_ANGLE_ACT > ANGLE_ACT && dead_zone_adj1 == 0)
					{
						gear2_adj = 1;	//从叶片2挡位使能（主叶片2挡位去能）
					}
					else if (ANOTHER_ANGLE_ACT <= ANGLE_ACT)
					{
						gear2_adj = 0;	//从叶片2挡位使去能（主叶片2挡位使能）
						dead_zone_adj1 = 0;
						dead_zone_adj2 = 0;
					}
				}
				else if (abs(ANGLE_ACT - ANOTHER_ANGLE_ACT) < 200 && dead_zone_adj2 == 0 && ANOTHER_ANGLE_ACT > 12000)//若主从叶片角度差小于0.2度且从叶片角度大于12度
				{
					gear2_adj = 2;		//从叶片按照电机每10-20秒/圈，进行微调调小（主叶片按照电机每10-20秒/圈，进行微调调小）。
				}
				else if (ANOTHER_ANGLE_ACT < 12000)	//若从叶片角度小于12度，关停
				{
					gear2_adj = 0;
					dead_zone_adj1 = 0;
					dead_zone_adj2 = 0;
				}
			}
			else if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && (fir_class_flag == 1) && (ANOTHER_ANGLE_ACT > 12000))//若应进入停止区 3区
			{
				gear1_adj = STATUS_Unvalid;	//1挡位区域挡位调整无效
				gear2_adj = STATUS_Unvalid;	//2挡位区域挡位调整无效
				gear4_adj = STATUS_Unvalid;	//4挡位区域挡位调整无效

				if (abs(ANOTHER_ANGLE_ACT - ANGLE_ACT) > 200)	//若主从叶片角度差大于200
				{
												//从叶片3挡位去能（主叶片4挡位去能）
					if (ANOTHER_ANGLE_ACT > ANGLE_ACT && ano_dead_zone_po == 0) //若从叶片角度大于主叶片角度
					{
						gear3_adj = 1;
					}
					else if (ANOTHER_ANGLE_ACT <= ANGLE_ACT)//若从叶片角度小于等于主叶片角度
					{
						gear3_adj = 0;
						ano_dead_zone_po = 0;	//离开该区间，电机波形已发送标志复位
					}
				}
				else if (abs(ANOTHER_ANGLE_ACT - ANGLE_ACT) < 200)//若主从叶片角度差小于200
				{
					gear3_adj = 0;
					ano_dead_zone_po = 0;
					ano_dead_zone_ne = 0;
				}
			}
			else if (ANO_Press_Valiad >= 2550 && fir_class_flag == 1)//若应进入开大区 4区
			{
				gear1_adj = STATUS_Unvalid;	//1挡位区域挡位调整无效
				gear2_adj = STATUS_Unvalid;	//2挡位区域挡位调整无效
				gear3_adj = STATUS_Unvalid;	//3挡位区域挡位调整无效

				gear4_adj = 0;				//从叶片4挡位使能（主叶片4挡位使能）
			}

			/*********************************挡位执行********************************************/
			/************进入最小开度区 1区************/
			if (ANO_Press_Valiad <= 2200 && MOTOR_FLAG == 0 && gear1_adj == 0 && fir_class_flag == 1)//开机上电时从叶片处在1区
			{
				MOTOR_FLAG_Slim = STATUS_Valid;		//2挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;		//4挡位区域复位

				dead_zone_adj1 = 0;					//2区叶片电机使能标志位（仅主叶片微调）复位
				dead_zone_adj2 = 0;					//2区叶片电机使能标志位（主从叶片微调）复位

				ano_dead_zone_ne = 0;				//3区从叶片电机开始正转标志位复位
				ano_dead_zone_po = 0;				//3区从叶片电机开始反转标志位复位

				ano_infro_valiad_ex_ne = 0;			//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;			//4区从叶片开始正转标志复位

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF = (ANOTHER_ANGLE_ACT - 12000);//计算1区时从叶片偏离角度

				if (ANOTHER_ANGLE_REF < 0 && ano_infro_valiad_po == 0 && abs(ANOTHER_ANGLE_REF) > 200)//从叶片电机第一次启动 & 角度小于12度 & 角度偏离0.2度以上
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_po = 1;
				}
				else if (ANOTHER_ANGLE_REF > 0 && ano_infro_valiad_ne == 0 && abs(ANOTHER_ANGLE_REF) > 200)//从叶片电机第一次启动 & 角度大于12度 & 角度偏离0.2度以上
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 42 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ne = 1;
				}
				else if ((abs(ANOTHER_ANGLE_REF) < 200))//从叶片偏离角度在0.2度之内 & 经过正转反转调整 & 主叶片电动机关停
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					MOTOR_FLAG = 1;

					ano_infro_valiad_po = 0;	//复位
					ano_infro_valiad_ne = 0;	//复位
				}
			}
			else if (ANO_Press_Valiad <= 2200 && MOTOR_FLAG == 1 && gear1_adj == 0 && fir_class_flag == 1)//电机运转时从叶片进入1区
			{
				MOTOR_FLAG_Slim = STATUS_Valid;		//2挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;		//4挡位区域复位

				dead_zone_adj1 = 0;					//2区叶片电机使能标志位（仅主叶片微调）复位
				dead_zone_adj2 = 0;					//2区叶片电机使能标志位（主从叶片微调）复位

				ano_dead_zone_ne = 0;				//3区从叶片电机开始正转标志位复位
				ano_dead_zone_po = 0;				//3区从叶片电机开始反转标志位复位

				ano_infro_valiad_ex_ne = 0;			//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;			//4区从叶片开始正转标志复位

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF_DET = (ANOTHER_ANGLE_ACT - 11800); //计算1区时从叶片偏离角度，按11.8度算，距离11.6度留有0.2的裕度
				if (ANOTHER_ANGLE_REF_DET < 0 && ano_infro_valiad_det_po == 0 && abs(ANOTHER_ANGLE_REF_DET) > 100)//从叶片角度小于11.8度 & 角度偏离0.1度以上
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 168 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 5hz
					ano_infro_valiad_det_po = 1;
				}
				else if (ANOTHER_ANGLE_REF_DET > 0 && ano_infro_valiad_det_ne == 0 && abs(ANOTHER_ANGLE_REF_DET) > 100)//从叶片进入1区 & 角度大于11.8度 & 角度偏离0.1度以上
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 168 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 5hz
					ano_infro_valiad_det_ne = 1;
				}
				else if ((abs(ANOTHER_ANGLE_REF_DET) < 100) && ((ano_infro_valiad_det_ne == 1) || (ano_infro_valiad_det_po == 1)))//从叶片偏离角度在0.1度之内 & 经过正转反转调整 & 主叶片电动机关停
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					ano_infro_valiad_det_ne = 0;	//复位
					ano_infro_valiad_det_po = 0;	//复位
				}
			}

			/************进入开大区 4区***************/
			if (ANO_Press_Valiad >= 2550 && MOTOR_FLAG_Ref == 0 && gear4_adj == 0 && fir_class_flag == 1)
			{
				MOTOR_FLAG = STATUS_Valid;		//1挡位区域复位
				MOTOR_FLAG_Slim = STATUS_Valid;	//2挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;//3挡位区域复位

				ano_infro_valiad_po = 0;		//1区从叶片开始正转标志（12度）复位
				ano_infro_valiad_ne = 0;		//1区从叶片开始反转标志（12度）复位
				ano_infro_valiad_det_po = 0;	//1区从叶片开始正转标志（11.6度）复位
				ano_infro_valiad_det_ne = 0;	//1区从叶片开始反转标志（11.6度）复位

				dead_zone_adj1 = 0;				//2区从叶片电机使能标志位（仅从叶片微调）复位
				dead_zone_adj2 = 0;				//2区从叶片电机使能标志位（主从叶片微调）复位

				ano_dead_zone_ne = 0;			//3区从叶片电机开始正转标志位复位
				ano_dead_zone_po = 0;			//3区从叶片电机开始反转标志位复位

				tim_another_flag = 0;

				ANOTHER_ANGLE_REF_EX = (ANOTHER_ANGLE_ACT - 16500);//计算4区时主叶片偏离角度
				if (ANOTHER_ANGLE_REF_EX < 0 && ano_infro_valiad_ex_po == 0 && abs(ANOTHER_ANGLE_REF_EX) > 200)//从叶片小于16.5度，但偏离角度大于0.2度
				{
					UPorDOWN = 0; // 0
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 21 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ex_po = 1;
				}
				else if (ANOTHER_ANGLE_REF_EX > 0 && ano_infro_valiad_ex_ne == 0 && abs(ANOTHER_ANGLE_REF_EX) > 200)//从叶片大于16.5度，但偏离角度大于0.2度
				{
					UPorDOWN = 1; // 1
					PWM_CTRL_SL(0);
					vTaskDelay(5);
					Motor_another_Start(50000 - 1, 21 - 1, UPorDOWN, 25000, 25000, 25000, 25000, 25000); // 20hz
					ano_infro_valiad_ex_ne = 1;
				}
				else if (abs(ANOTHER_ANGLE_REF_EX) < 200)//从叶片大于或小于16.5度，但偏离角度小于0.2度
				{
					TIM_Another_Shutdown();
					PWM_CTRL_SL(1);

					ano_infro_valiad_ex_ne = 0;
					ano_infro_valiad_ex_po = 0;
				}
			}

			/************进入微调区 2区***************/
			if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 1 && dead_zone_adj1 == 0 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))//仅从叶片微调调小
			{
				MOTOR_FLAG = STATUS_Valid;			//1挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;		//4挡位区域复位

				ano_infro_valiad_po = 0;			//1区从叶片开始正转标志（12度）复位
				ano_infro_valiad_ne = 0;			//1区从叶片开始反转标志（12度）复位
				ano_infro_valiad_det_po = 0;		//1区从叶片开始正转标志（11.6度）复位
				ano_infro_valiad_det_ne = 0;		//1区从叶片开始反转标志（11.6度）复位

				ano_dead_zone_ne = 0;				//3区从叶片电机开始正转标志位复位
				ano_dead_zone_po = 0;				//3区从叶片电机开始反转标志位复位

				ano_infro_valiad_ex_ne = 0;			//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;			//4区从叶片开始正转标志复位

				tim_another_flag = 0;

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				dead_zone_adj1 = 1;
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 2 && dead_zone_adj2 == 0 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))//从叶片微调调小（主叶片也微调调小）
			{
				MOTOR_FLAG = STATUS_Valid;			//1挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;		//4挡位区域复位

				ano_infro_valiad_po = 0;		//1区从叶片开始正转标志（12度）复位
				ano_infro_valiad_ne = 0;		//1区从叶片开始反转标志（12度）复位
				ano_infro_valiad_det_po = 0;	//1区从叶片开始正转标志（11.6度）复位
				ano_infro_valiad_det_ne = 0;	//1区从叶片开始反转标志（11.6度）复位

				ano_dead_zone_ne = 0;			//3区从叶片电机开始正转标志位复位
				ano_dead_zone_po = 0;			//3区从叶片电机开始反转标志位复位

				ano_infro_valiad_ex_ne = 0;		//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;		//4区从叶片开始正转标志复位

				tim_another_flag = 0;

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				dead_zone_adj2 = 1;
			}
			else if (ANO_Press_Valiad > 2220 && ANO_Press_Valiad <= 2460 && MOTOR_FLAG_Slim == 0 && gear2_adj == 0 && fir_class_flag == 1)//从叶片停转
			{
				MOTOR_FLAG = STATUS_Valid;			//1挡位区域复位
				MOTOR_FLAG_UNVAD = STATUS_Valid;	//3挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;		//4挡位区域复位

				TIM_Another_Shutdown();
				PWM_CTRL_SL(1);

				dead_zone_adj1 = 0;					//复位
				dead_zone_adj2 = 0;					//复位
			}

			/************进入停止区 3区***************/
			if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && MOTOR_FLAG_UNVAD == 0 && gear3_adj == 1 && fir_class_flag == 1 && (ANOTHER_ANGLE_ACT > 12000))
			{
				MOTOR_FLAG = STATUS_Valid; 		//1挡位区域复位
				MOTOR_FLAG_Slim = STATUS_Valid;	//2挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;	//4挡位区域复位

				ano_infro_valiad_po = 0;		//1区从叶片开始正转标志（12度）复位
				ano_infro_valiad_ne = 0;		//1区从叶片开始反转标志（12度）复位
				ano_infro_valiad_det_po = 0;	//1区从叶片开始正转标志（11.6度）复位
				ano_infro_valiad_det_ne = 0;	//1区从叶片开始反转标志（11.6度）复位

				dead_zone_adj1 = 0;				//2区从叶片电机使能标志位（仅从叶片微调）复位
				dead_zone_adj2 = 0;				//2区从叶片电机使能标志位（主从叶片微调）复位

				ano_infro_valiad_ex_ne = 0;		//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;		//4区从叶片开始正转标志复位

				PWM_CTRL_SL(0);
				vTaskDelay(5);
				Motor_another_Start(50000 - 1, 168 - 1, 1, 25000, 25000, 25000, 25000, 25000); // 5HZ
				ano_dead_zone_po = 1;

			}
			else if (ANO_Press_Valiad > 2460 && ANO_Press_Valiad < 2550 && MOTOR_FLAG_UNVAD == 0 && gear3_adj == 0 && fir_class_flag == 1)
			{
				MOTOR_FLAG = STATUS_Valid; 		//1挡位区域复位
				MOTOR_FLAG_Slim = STATUS_Valid;	//2挡位区域复位
				MOTOR_FLAG_Ref = STATUS_Valid;	//4挡位区域复位

				ano_infro_valiad_po = 0;		//1区从叶片开始正转标志（12度）复位
				ano_infro_valiad_ne = 0;		//1区从叶片开始反转标志（12度）复位
				ano_infro_valiad_det_po = 0;	//1区从叶片开始正转标志（11.6度）复位
				ano_infro_valiad_det_ne = 0;	//1区从叶片开始反转标志（11.6度）复位

				dead_zone_adj1 = 0;				//2区从叶片电机使能标志位（仅从叶片微调）复位
				dead_zone_adj2 = 0;				//2区从叶片电机使能标志位（主从叶片微调）复位

				ano_infro_valiad_ex_ne = 0;		//4区从叶片开始反转标志复位
				ano_infro_valiad_ex_po = 0;		//4区从叶片开始正转标志复位

				TIM_Another_Shutdown();
				PWM_CTRL_SL(1);

				ano_dead_zone_po = 0;			//复位
				ano_dead_zone_ne = 0;			//复位
			}
		}
		//*******************************开环*********************************************//
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

			Motor_another_Start(50000 - 1, 168 - 1, plusorminu, 25000, 25000, 25000, 25000, 25000); // 5HZ        //50%占空比 8V //按道理来说应该置tim_another_flag =1
		}
		else if (2 == p_gSysUserRegs->CAN2_Value[4])
		{
			Motor_another_Stop = 2; //电机停止标志位
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
