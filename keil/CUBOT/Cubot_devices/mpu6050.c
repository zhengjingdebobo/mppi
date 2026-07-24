#include "mpu6050.h"
#include "drv_iic.h"
#include "math.h"
#include "kalman.h" 
#include "holder.h"
#include "gpio.h"
#include "referee.h"

MPUAngle mpuAngle;
SENSOR_StoreTypeDef sensor={0};
gyro_data_t  gyro_data=
{
	.sens_yaw  =0.161, // yaw   角速度积分为角度的值
	.sens_pitch=0.6783  // pitch 角速度积分为角度的值  但是没有使用
};
/**
  * @brief   写数据到MPU6050寄存器
  */
uint8_t MPU6050_WriteByte(uint8_t reg_add, uint8_t reg_dat)
{
	return Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat);
}


/**
  * @brief   从MPU6050寄存器读取数据
  */
uint8_t MPU6050_ReadData(uint8_t reg_add, unsigned char* Read, uint8_t num)
{
	return Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read);
}

/**
  * @brief   电源开启
  */
void MPU6050_PowerOn(void)
{
	//< MPU6050 电源开启
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
  * @brief   电源关闭
  */
void MPU6050_PowerOff(void)
{
	//< MPU6050 电源关闭
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

}


/**
  * @brief   初始化MPU6050
  */
void MPU6050_Init(void)
{
	uint8_t TAddr;
	do
	{
		MPU6050_WriteByte(0x6B, 0x01);  	   //休眠
		HAL_Delay(500);
		MPU6050_WriteByte(0x6B, 0x00);  	   //解除休眠状态0x00
		HAL_Delay(50);	
		
		MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &TAddr, 1);		
	}while(TAddr!=0x68);
	

  MPU6050_WriteByte( 0x6B, 0x00);    //解除休眠状态0x00
	HAL_Delay(50);
	MPU6050_WriteByte(0x19, 0x00);     //采样频率（1KHz）
	HAL_Delay(50);
	MPU6050_WriteByte(0x1A, 0x03);     //低通滤波0x00
	HAL_Delay(50);
	MPU6050_WriteByte(0x1B, 0x10);    //陀螺仪量程 
	HAL_Delay(50);
	MPU6050_WriteByte(0x1C, 0x09);   //加速度量程 
	HAL_Delay(50);
}

/**
  * @brief   读取MPU6050的ID
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
	MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1);    //读器件地址
	if (Re != 0x68)
	{	//		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return Re;
	}
	else
	{	//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return Re;
	}
}

/**
  * @brief   读取陀螺仪数据
  */
float Gyro_File_Buf[3][GYRO_FILTER_NUM];
void MPU6050_RawDataUpdate(MPUAngle* mpuAngle) 
{
	uint8_t accBuf[6];
	uint8_t gyroBuf[6];
	MPU6050_ReadData(MPU6050_ACC_OUT, accBuf, 6);
	mpuAngle->accRaw.x = (accBuf[0] << 8) | accBuf[1];
	mpuAngle->accRaw.y = (accBuf[2] << 8) | accBuf[3];
	mpuAngle->accRaw.z = (accBuf[4] << 8) | accBuf[5];

	MPU6050_ReadData(MPU6050_GYRO_OUT, gyroBuf, 6);
	mpuAngle->gyroRaw.x = (gyroBuf[0] << 8) | gyroBuf[1];
	mpuAngle->gyroRaw.y = (gyroBuf[2] << 8) | gyroBuf[3];
	mpuAngle->gyroRaw.z = (gyroBuf[4] << 8) | gyroBuf[5];
}


/**********************************************************************************
 ==============================================================================
						  卡尔曼滤波+四元数解算
 ==============================================================================
 ********************************************************************************/

/**
  * @brief   数据处理
  */
void PrepareForIMU(void)
{
	float sumx,sumy,sumz;//sum_yaw		
	static uint8_t gyro_filter_cnt = 0;
	int i =0;
				
		sensor.acc.origin.x = mpuAngle.accRaw.x;
		sensor.acc.origin.y = mpuAngle.accRaw.y;
		sensor.acc.origin.z = mpuAngle.accRaw.z;
		
//修改
		sensor.gyro.origin.x = mpuAngle.gyroRaw.x- (int16_t)sensor.gyro.quiet.x;
		sensor.gyro.origin.y = mpuAngle.gyroRaw.y- (int16_t)sensor.gyro.quiet.y;
		sensor.gyro.origin.z = mpuAngle.gyroRaw.z- (int16_t)sensor.gyro.quiet.z;

		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
			
			sumx = 0;
			sumy = 0;
			sumz = 0;
			
		for(i=0;i<GYRO_FILTER_NUM;i++)
		{
			sumx += Gyro_File_Buf[0][i];
			sumy += Gyro_File_Buf[1][i];
			sumz += Gyro_File_Buf[2][i];
		}
		
		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
		
		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;//弧度/s
		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.user.x  = sumx / (float)GYRO_FILTER_NUM;// * Gyro_Gr;  //
		sensor.gyro.user.y  = sumy / (float)GYRO_FILTER_NUM;// * Gyro_Gr;
		sensor.gyro.user.z  = sumz / (float)GYRO_FILTER_NUM;// * Gyro_Gr;
		
		sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC X轴卡尔曼滤波
		sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC Y轴卡尔曼滤波
		sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC Z轴卡尔曼滤波
	
}



/**
  * @brief   获得时间戳
  */
uint32_t Get_Time_Micros(void)
{
	return TIM4->CNT;
}

/**
  * @brief   快速求平方根算法
  */
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
  * @brief   求解四元数
  */
volatile uint32_t lastUpdate, now;                // 采样周期计数 单位 us
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;            // scaled integral error

void IMUupdate(SENSOR_StoreTypeDef *sensor)
{
     float norm;
//    float hx, hy, hz, bx, bz;
    float vx, vy, vz;//, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
//    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
//    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   
	
	  now = Get_Time_Micros();  //读取时间 单位是us   
	
    if(now<lastUpdate)
    {
			halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);   // 1/2000000=0.0000005=42000000/84
    }
    else	
    {
       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间

    //快速求平方根算法
    norm = invSqrt(sensor->acc.averag.x*sensor->acc.averag.x 
									+ sensor->acc.averag.y*sensor->acc.averag.y 
									+	sensor->acc.averag.z*sensor->acc.averag.z);       
    sensor->acc.averag.x = sensor->acc.averag.x * norm;
    sensor->acc.averag.y = sensor->acc.averag.y * norm;
    sensor->acc.averag.z = sensor->acc.averag.z * norm;

    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
		//vz = 1 - 2*q1q1 - 2*q2q2;

    ex = (sensor->acc.averag.y * vz - sensor->acc.averag.z * vy);// + (my*wz - mz*wy);
    ey = (sensor->acc.averag.z * vx - sensor->acc.averag.x * vz);// + (mz*wx - mx*wz);
    ez = (sensor->acc.averag.x * vy - sensor->acc.averag.y * vx);// + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
			exInt = exInt + ex * Kii * halfT;
			eyInt = eyInt + ey * Kii * halfT;	
			ezInt = ezInt + ez * Kii * halfT;
			// 用叉积误差来做PI修正陀螺零偏
			sensor->gyro.radian.x = sensor->gyro.radian.x + Kp*ex + exInt;
			sensor->gyro.radian.y = sensor->gyro.radian.y + Kp*ey + eyInt;
			sensor->gyro.radian.z = sensor->gyro.radian.z + Kp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*sensor->gyro.radian.x - q2*sensor->gyro.radian.y - q3*sensor->gyro.radian.z)*halfT;
    tempq1 = q1 + (q0*sensor->gyro.radian.x + q2*sensor->gyro.radian.z - q3*sensor->gyro.radian.y)*halfT;
    tempq2 = q2 + (q0*sensor->gyro.radian.y - q1*sensor->gyro.radian.z + q3*sensor->gyro.radian.x)*halfT;
    tempq3 = q3 + (q0*sensor->gyro.radian.z + q1*sensor->gyro.radian.y - q2*sensor->gyro.radian.x)*halfT;  

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
	
		//>
	//	mpuAngle.yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
  //  mpuAngle.roll= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
    mpuAngle.pitch= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 
	}	
/**
  * @brief   防止零漂
  */	
uint8_t Gyro_OFFEST(SENSOR_StoreTypeDef *sensor)
{
int cnt_g=1000;
	int cnt = cnt_g;
	int cnt_err_x=0;
	int cnt_err_y=0;
	int cnt_err_z=0;
	
	int16_t x_last,y_last,z_last;
	 float  tempgx=0,tempgy=0,tempgz=0;
	static int err_cnt=0;
	int err;
	
	err=err_cnt/5;

	x_last = mpuAngle.gyroRaw.x;
	y_last = mpuAngle.gyroRaw.y;
	z_last = mpuAngle.gyroRaw.z;
	
	while(cnt_g--)       //循环采集1000次   求平均
	{

	 	MPU6050_RawDataUpdate(&mpuAngle);
		
		sensor->gyro.origin.x = mpuAngle.gyroRaw.x;
		sensor->gyro.origin.y = mpuAngle.gyroRaw.y;
		sensor->gyro.origin.z = mpuAngle.gyroRaw.z;
		
	 
		if(abs(sensor->gyro.origin.x-x_last)>=10+err)
			cnt_err_x++;
		if(abs(sensor->gyro.origin.y-y_last)>=10+err)
			cnt_err_y++;
		if(abs(sensor->gyro.origin.z-z_last)>=10+err)
			cnt_err_z++;
				
		tempgx+= sensor->gyro.origin.x;
		tempgy+= sensor->gyro.origin.y;
		tempgz+= sensor->gyro.origin.z;

		if((cnt_err_x>=50)||(cnt_err_y>=50)||(cnt_err_z>=50))
		{
			err_cnt++;
			return 1;
		}
	}		
	sensor->gyro.quiet.x=tempgx/cnt;
	sensor->gyro.quiet.y=tempgy/cnt;
	sensor->gyro.quiet.z=tempgz/cnt;	
	return 0;
}


/**********************************************************************************
 ==============================================================================
						                陀螺仪外部接口函数
 ==============================================================================
***********************************************************************************/
	
void IMU_Boot(void)	
{
	while(Gyro_OFFEST(&sensor)==1);	
}

void Gyro_init(void)
{
	//>上电
		MPU6050_PowerOn();
	//>初始化
	  MPU6050_Init();
	//>计数为 0
	  gyro_data.cnt=0;
}

void Gyro_reset(void)
{
	//>下电
	  MPU6050_PowerOff();
	  HAL_Delay(50);
	//>上电初始化
    Gyro_init();
}

void Gyro_data_update(void)
{
	//>数据更新次数累加
  gyro_data.cnt++;
	
	MPU6050_RawDataUpdate(&mpuAngle);
	
	PrepareForIMU();
	IMUupdate(&sensor);
	
	if(gyro_data.cnt>500)
	{
	
			gyro_data.gyro_w_yaw   = sensor.gyro.user.z;
			gyro_data.gyro_w_pitch = sensor.gyro.user.x;

			if((abs(gyro_data.gyro_w_yaw)>GYRO_GAP)&&referee2022.game_robot_status.mains_power_chassis_output == 1)
			{
				gyro_data.yaw_speed=((gyro_data.gyro_w_yaw)*0.0001f*gyro_data.sens_yaw);
				gyro_data.yaw_angle=gyro_data.yaw_angle + gyro_data.yaw_speed;
			}
			
			if((abs(gyro_data.gyro_w_pitch)>GYRO_GAP)&&referee2022.game_robot_status.mains_power_chassis_output == 1)
			{
				gyro_data.pitch_speed=((gyro_data.gyro_w_pitch)*0.0001f*gyro_data.sens_yaw);
			}

			
			
			gyro_data.pitch_angle = mpuAngle.pitch;
			
			if(Hero_Holder.Flag.Reset_OK == 0)
			{
				gyro_data.yaw_angle = 0;

			}
		
  }

}




