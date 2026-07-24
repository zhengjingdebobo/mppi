#ifndef DR16_H
#define DR16_H

//#include "stm32h7xx_hal.h"

//#define Key_Filter_Num 7       //< 按键检测消抖滤波时间(ms)


///**
//  * @brief  详细按键定义，补充接收机结构体内容
//  */
//#define key_W        key[0]
//#define key_A        key[1]
//#define key_S        key[2]
//#define key_D        key[3]
//#define key_shift    key[4]
//#define key_ctrl     key[5]
//#define key_Q        key[6]
//#define key_E        key[7]
//#define key_V        key[8]
//#define key_F        key[9]
//#define key_G        key[10]
//#define key_C        key[11]
//#define key_R        key[12]
//#define key_B        key[13]
//#define key_Z        key[14]
//#define key_X        key[15]
// 
//#define key_W_flag        keyflag[0]
//#define key_A_flag        keyflag[1]
//#define key_S_flag        keyflag[2]
//#define key_D_flag        keyflag[3]
//#define key_shift_flag    keyflag[4]
//#define key_ctrl_flag     keyflag[5]
//#define key_Q_flag        keyflag[6]
//#define key_E_flag        keyflag[7]
//#define key_V_flag        keyflag[8]
//#define key_F_flag        keyflag[9]     // %2
//#define key_G_flag        keyflag[10]
//#define key_C_flag        keyflag[11]    // %2
//#define key_R_flag        keyflag[12]    // %2
//#define key_B_flag        keyflag[13]
//#define key_Z_flag        keyflag[14]
//#define key_X_flag        keyflag[15]

//#define laste_key_W_flag        last_keyflag[0]
//#define laste_key_A_flag        last_keyflag[1]
//#define laste_key_S_flag        last_keyflag[2]
//#define laste_key_D_flag        last_keyflag[3]
//#define laste_key_shift_flag    last_keyflag[4]
//#define laste_key_ctrl_flag     last_keyflag[5]
//#define laste_key_Q_flag        last_keyflag[6]
//#define laste_key_E_flag        last_keyflag[7]
//#define laste_key_V_flag        last_keyflag[8]
//#define laste_key_F_flag        last_keyflag[9]     // %2
//#define laste_key_G_flag        last_keyflag[10]
//#define laste_key_C_flag        last_keyflag[11]    // %2
//#define laste_key_R_flag        last_keyflag[12]    // %2
//#define laste_key_B_flag        last_keyflag[13]
//#define laste_key_Z_flag        last_keyflag[14]
//#define laste_key_X_flag        last_keyflag[15] 

//#define W_Num        0
//#define A_Num        1
//#define S_Num        2
//#define D_Num        3
//#define shift_Num    4
//#define ctrl_Num     5
//#define Q_Num        6
//#define E_Num        7
//#define V_Num        8
//#define F_Num        9
//#define G_Num        10
//#define C_Num        11
//#define R_Num        12
//#define B_Num        13
//#define Z_Num        14
//#define X_Num        15


///**
//  * @brief  接收机接收数据类型, 包含rc遥控器数据、mouse鼠标数据和 keyflag 按键数据
//  */
//typedef struct { 
// struct {       
//	 uint16_t sw;
//	 uint16_t ch0;       
//	 uint16_t ch1;       
//	 uint16_t ch2;      
//	 uint16_t ch3;      
//	 uint8_t  s1; 
//	 uint8_t  s2;	
//	 uint8_t  s1_last; 
//	 uint8_t  s2_last; 
//	}rc; 

// struct {       
//	 int16_t x;       
//	 int16_t y;       
//	 int16_t z;         
//	 uint8_t press_l;     
//	 uint8_t press_r; 
//	 uint8_t press_l_flag;  
//   uint8_t last_press_l_flag; 
//	 uint8_t press_r_flag; 
//	 uint8_t last_press_r_flag; 
// }mouse; 

//	struct {     
//		uint16_t v;  
//	}keyboard;
//	
//	uint8_t  key[18];   
//	uint8_t  keyflag[18];	
//	uint8_t  last_keyflag[18];
//	uint8_t  isUnpackaging;  	 //< 解算状态标志位，解算过程中不读取数据
//	uint8_t  isOnline;
//	uint32_t onlineCheckCnt;
//	uint16_t a;
//	uint16_t b;
//	uint16_t c;
//	uint16_t d;
//	float Chassis_Y_Integ;//斜坡积分变量
//  float Chassis_X_Integ;
//}RC_Ctrl; 


///**
//  * @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
//	* @param[in] RC_Ctl  接收机数据类型首地址，储存杆量和按键信息
//  * @retval    RC_Ctl  接收机数据类型首地址，储存杆量和按键信息
//  */
//void DR16Init(RC_Ctrl* RC_Ctl);


///**
//  * @brief  按键消抖，检测是否为有效按下
//	* @param[in] RC_Ctl  接收机数据类型首地址
//  * @retval    RC_Ctl  接收机数据类型首地址
//  */
//void PC_keybroad_filter(RC_Ctrl* RC_Ctl);


///**
//   * @brief  创建dr16的接收机缓存数组, 并对全局变量rc_Ctrl赋值，以供其他函数调用
//   */
//void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer);


//extern RC_Ctrl rc_Ctrl;

#endif
