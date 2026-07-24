#ifndef MPU_6050_H
#define MPU_6050_H
#include "stm32h7xx_hal.h"

// MPU6050, Standard address 0x68
#define MPU6050_ADDRESS         0xD0
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_SMPLRT_DIV      0  //8000Hz
#define MPU6050_DLPF_CFG        0
#define MPU6050_GYRO_OUT        0x43     //MPU6050陀螺仪数据寄存器地址
#define MPU6050_ACC_OUT         0x3B     //MPU6050加速度数据寄存器地址

#define MPU6050_RA_WHO_AM_I         0x75

#define MPU_ERROR 		I2C_ERROR
#define MPU_INFO 		  I2C_INFO
#define q30  1073741824.0f

#define GYRO_FILTER_NUM 8


#define GYRO_GAP 50
#define K_ANGLESPEED_2_ANGLE 0.000018f //陀螺仪积分获得角度系数


#define Kp 0.5f         //原始1.0f               // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kii 0.1f                     // integral gain governs rate of convergence of gyroscope biases
#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制	
#define Gyro_Gr		0.0005327f   //  1/32768*1000/57.3 
/**********************焦写的结构体*************************/
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}_int16_t;


typedef struct
{
	_int16_t accRaw;
	_int16_t gyroRaw;
	int sensor_temp;
  uint32_t sensor_time;
	
	float yaw;
	float roll;
	float pitch;

}MPUAngle;

struct _float{
	      float x;
				float y;
				float z;};

struct _double{
       double x;
	     double y;
	     double z;};		

struct _trans{
     struct _double origin;  //原始值   就是直接读取mpu6050的数据  acc 和 gyro要减去静态值
	   struct _double averag;  //平均值   进行卡尔曼滤波之后的值     acc才有          
	   struct _double histor;  //历史值   没有使用
	   struct _double quiet;   //静态值   进行1000次求和取平均值             gyro才有
	   struct _double radian;  //弧度值   角度转化为弧度                     gyro才有               
		 struct _double user;   //未经adc转换值
          };
/**
	*@biref SENSOR_StoreTypeDef#传感器数据结构体，用于将获取的数据进行运算
					*/
typedef struct {   
	struct _trans acc;
	struct _trans gyro;
}SENSOR_StoreTypeDef;


typedef struct 
{

  int16_t gyro_w_yaw;  
	int16_t gyro_w_pitch;
	float yaw_angle;
	float yaw_speed;
  float pitch_angle;
	float pitch_speed;
  float sens_pitch;
	float sens_yaw;
  uint32_t cnt;
  

}gyro_data_t;



extern MPUAngle mpuAngle;
extern SENSOR_StoreTypeDef sensor;
extern gyro_data_t  gyro_data;







/*********************************************************/
/**
  * @brief   写数据到MPU6050寄存器
  * @param   reg_add:寄存器地址
	* @param	 reg_data:要写入的数据
  */
uint8_t MPU6050_WriteByte(uint8_t reg_add, uint8_t reg_dat);
/**
  * @brief   从MPU6050寄存器读取数据
  * @param   reg_add:寄存器地址
	* @param	 Read：存储数据的缓冲区
	* @param	 num：要读取的数据量
  */
uint8_t MPU6050_ReadData(uint8_t reg_add, unsigned char* Read, uint8_t num);
/*********************************************************/


//void MPU6050_RawDataUpdate(MPUAngle* mpuAngle);

void MPU6050_PowerOn(void);
void MPU6050_PowerOff(void);
void MPU6050_Init(void);                        //初始化
void MPU6050_RawDataUpdate(MPUAngle* mpuAngle); //读取陀螺仪数据
void PrepareForIMU(void);                 //陀螺仪数据处理



void IMU_Boot(void);
void Gyro_init(void);
void Gyro_reset(void);
void Gyro_data_update(void);
#endif


