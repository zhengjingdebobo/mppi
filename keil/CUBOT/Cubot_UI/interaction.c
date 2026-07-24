#include "interaction.h"
#include "string.h"
#include "usart.h"
#include "brain.h"





/*

			 ***************代码解析*****************
	Data_Pack[n]用于存放我们发给裁判系统的数据，该数据通过串口3发送给裁判系统，裁判系统
	依据协议将在操作手界面上显示用户自定义UI界面。
	Data_Pack[0]-Data_Pack[4]是帧头，其中的数据含义在裁判系统协议手册里开头给出，我们需
	要改的就只是Data_Pack[1]和Data_Pack[2]这两位，这两位用于存放数据长度，绘制不同的图形
	所需的数据长度是不一样的：

	内容ID				数据长度				功能
	0x0101					21				绘制一个图形
	0x0102					36				绘制两个图形
	0x0103					81				绘制五个图形
	0x0104					111				绘制七个图形
	0x0110					51				绘制字符图形

	其中，每个图形所占数据位为15位，字符占45位，数据段ID内容、发送者ID、接收者ID各占2位，也就是数据
	的前六位用于存放ID信息，所以21=6+15,36=6+15*2,依次类推。

	首先建立一个图形结构体用于存放图形数据，在定义相关功能函数将结构体发送出去，完成UI界面的显示


				**************使用方法****************
	将想要显示的图形函数放在函数Client_Display（）里，并且在主任务函数里不停调用Client_Display（）函数
 
 */
 
 uint8_t Data_Pack[128]__attribute__((at(0x24005000)));

ext_client_custom_character_t ext_client_custom_character;
ext_client_custom_graphic_single_t ext_client_custom_graphic_float;
ext_client_custom_graphic_single_t ext_client_custom_graphic_int;
ext_client_custom_graphic_single_t ext_client_custom_graphic_line;
ext_client_custom_graphic_single_t ext_client_custom_graphic_circle;
ext_client_custom_graphic_single_t ext_client_custom_graphic_rectangle;
ext_client_custom_graphic_single_t ext_client_custom_graphic_ellipe;
ext_client_custom_graphic_single_t ext_client_custom_graphic_arc;
ext_client_custom_graphic_five_t ext_client_custom_graphic_patterning;
ext_client_custom_graphic_five_t ext_client_custom_graphic_patterning_second;
ext_client_custom_graphic_five_t ext_client_custom_graphic_patterning_third;
ext_client_custom_graphic_seven_t ext_client_custom_graphic_patterning_fourth;
//ext_client_custom_graphic_seven_t ext_client_custom_graphic_patterning_fifth;
ext_client_graphic_delete_t ext_client_graphic_delete;

game_state_t each_state={
	1,1,1,0,1
};


void graphic_data_modify(graphic_data_struct_t* graphic_data_struct, 
												 uint8_t name,
												 uint32_t operation,
												 uint32_t type,
												 uint32_t layer,
												 uint32_t color,
												 uint32_t size,
												 uint32_t number,		//< 小数点后位数
												 uint32_t wide,
											   uint32_t start_x,  //开始X坐标
												 uint32_t start_y,  //开始Y坐标
												 uint32_t radius,   //半径或字体大小
												 uint32_t end_x,    //终止X坐标
												 uint32_t end_y)    //终止Y坐标
{
		graphic_data_struct->operate_tpye = operation;
	
}

void referee_graphic_delete(uint8_t del_layer, uint8_t operation, uint8_t robot_id)
{
	uint16_t crc16_temp;
	ext_client_graphic_delete.frame_header.SOF = 0xA5;
	ext_client_graphic_delete.frame_header.data_length = 8;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
	ext_client_graphic_delete.frame_header.seq = 0;
	ext_client_graphic_delete.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_character.frame_header,4,0xFF); 
	ext_client_graphic_delete.cmd_id = 0x0301;
	ext_client_graphic_delete.ext_student_interactive_header_data.data_cmd_id = 0x0100;
	ext_client_graphic_delete.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_graphic_delete.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_graphic_delete.ext_client_custom_graphic_delete.operate_tpye=operation;
	ext_client_graphic_delete.ext_client_custom_graphic_delete.layer=del_layer;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_graphic_delete, 15, 0xFFFF);
	ext_client_graphic_delete.CRC16[0] = crc16_temp & 0xFF;
	ext_client_graphic_delete.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_graphic_delete,17);
}


/**
  * @brief  在0号图层画字符的程序，最长30个字符
	*/
void referee_draw_string(uint8_t robot_id,char *string,uint8_t string_dex,uint8_t control_way,uint8_t color,uint8_t on_layer, uint8_t start_angle,	uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	ext_client_custom_character.frame_header.SOF = 0xA5;
	ext_client_custom_character.frame_header.data_length = 51;   //< sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)
	ext_client_custom_character.frame_header.seq = 0;
	ext_client_custom_character.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_character.frame_header,4,0xFF); 
	ext_client_custom_character.cmd_id = 0x0301;
	ext_client_custom_character.ext_student_interactive_header_data.data_cmd_id = 0x0110;
	ext_client_custom_character.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_character.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_character.grapic_data_struct.graphic_name[0]=string_dex;
	ext_client_custom_character.grapic_data_struct.graphic_name[1]=0x0;
	ext_client_custom_character.grapic_data_struct.graphic_name[2]=0x0;
	ext_client_custom_character.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_character.grapic_data_struct.graphic_tpye = 0x07;         //< 表明字符型
	ext_client_custom_character.grapic_data_struct.layer = on_layer;
	ext_client_custom_character.grapic_data_struct.color = color;
	ext_client_custom_character.grapic_data_struct.start_angle = start_angle;
	ext_client_custom_character.grapic_data_struct.end_angle = 20;
	ext_client_custom_character.grapic_data_struct.width = 2;
	ext_client_custom_character.grapic_data_struct.start_x = x;
	ext_client_custom_character.grapic_data_struct.start_y = y;
	ext_client_custom_character.grapic_data_struct.radius = 0;
	ext_client_custom_character.grapic_data_struct.end_x = 0;
	ext_client_custom_character.grapic_data_struct.end_y = 0;
	memset(ext_client_custom_character.data, 0, 30);					//< 每次绘制前清除上次字符串缓存
	strcpy((char *)ext_client_custom_character.data, string);
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_character, 58, 0xFFFF);
	ext_client_custom_character.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_character.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_character,60);
}


/**
  * @brief  在1号图层画浮点数的程序，保留三位小数
	*/
int32_t float_temp;
void referee_draw_float(uint8_t robot_id, float float_data,uint8_t float_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	float_temp =(int32_t)(float_data * 1000);
	ext_client_custom_graphic_float.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_float.frame_header.data_length = 21;
	ext_client_custom_graphic_float.frame_header.seq = 0;
	ext_client_custom_graphic_float.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_float.frame_header,4,0xFF); 
	ext_client_custom_graphic_float.cmd_id = 0x0301;
	ext_client_custom_graphic_float.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_float.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_float.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_float.grapic_data_struct.graphic_name[0]=float_index;
	ext_client_custom_graphic_float.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	ext_client_custom_graphic_float.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	ext_client_custom_graphic_float.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_float.grapic_data_struct.graphic_tpye = 5;
	ext_client_custom_graphic_float.grapic_data_struct.layer = layer;
	ext_client_custom_graphic_float.grapic_data_struct.color = color;
	ext_client_custom_graphic_float.grapic_data_struct.start_angle = 18;
	ext_client_custom_graphic_float.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	ext_client_custom_graphic_float.grapic_data_struct.width = 2;
	ext_client_custom_graphic_float.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_float.grapic_data_struct.start_y = y;

	ext_client_custom_graphic_float.grapic_data_struct.radius = float_temp & 0x3FF;        //< 低10位
	ext_client_custom_graphic_float.grapic_data_struct.end_x =  (float_temp>>10) & 0x7FF;	 //< 中11位
	ext_client_custom_graphic_float.grapic_data_struct.end_y =  float_temp>>21;						 //< 高11位
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_float, 28, 0xFFFF);
	ext_client_custom_graphic_float.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_float.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_float,30);
}


/**
  * @brief  在8号图层画整数的程序
	*/
void referee_draw_int(uint8_t robot_id,int32_t int_data,uint8_t int_index, uint8_t control_way, uint8_t color, uint16_t x, uint16_t y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_int.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_int.frame_header.data_length = 21;
	ext_client_custom_graphic_int.frame_header.seq = 0;
	ext_client_custom_graphic_int.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_int.frame_header,4,0xFF); 
	ext_client_custom_graphic_int.cmd_id = 0x0301;
	ext_client_custom_graphic_int.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_int.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_int.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_int.grapic_data_struct.graphic_name[0]=int_index;
	ext_client_custom_graphic_int.grapic_data_struct.graphic_name[1] = 0x0;     //< "A" 97
	ext_client_custom_graphic_int.grapic_data_struct.graphic_name[2]= 0x0;      //< "A"
	ext_client_custom_graphic_int.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_int.grapic_data_struct.graphic_tpye = 0x06;				//< 整形为06
	ext_client_custom_graphic_int.grapic_data_struct.layer = 8;
	ext_client_custom_graphic_int.grapic_data_struct.color = color;
	ext_client_custom_graphic_int.grapic_data_struct.start_angle = 18;
	ext_client_custom_graphic_int.grapic_data_struct.end_angle = 0x0;      			//< 小数位有效个数
	ext_client_custom_graphic_int.grapic_data_struct.width = 2;
	ext_client_custom_graphic_int.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_int.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_int.grapic_data_struct.radius = int_data & 0x3FF;        //< 低10位
	ext_client_custom_graphic_int.grapic_data_struct.end_x =  (int_data>>10) & 0x7FF;	 //< 中11位
	ext_client_custom_graphic_int.grapic_data_struct.end_y =  int_data>>21;						 //< 高11位

	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_int, 28, 0xFFFF);
	ext_client_custom_graphic_int.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_int.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_int,30);
}


/**
  * @brief  在5号图层画线的程序
	*/
void referee_draw_line(uint8_t robot_id, uint8_t line_index, uint8_t control_way,uint8_t color,uint16_t x,uint16_t y,uint16_t end_x,uint16_t end_y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_line.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_line.frame_header.data_length = 21;
	ext_client_custom_graphic_line.frame_header.seq = 0;
	ext_client_custom_graphic_line.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_line.frame_header,4,0xFF); 
	ext_client_custom_graphic_line.cmd_id = 0x0301;
	ext_client_custom_graphic_line.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_line.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_line.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_line.grapic_data_struct.graphic_name[0]=line_index;
	ext_client_custom_graphic_line.grapic_data_struct.graphic_name[1]=0x0;
	ext_client_custom_graphic_line.grapic_data_struct.graphic_name[2]=0x0;
	ext_client_custom_graphic_line.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_line.grapic_data_struct.graphic_tpye = 0x00;
	ext_client_custom_graphic_line.grapic_data_struct.layer = 5;
	ext_client_custom_graphic_line.grapic_data_struct.color = color;
	ext_client_custom_graphic_line.grapic_data_struct.start_angle = 0x0;
	ext_client_custom_graphic_line.grapic_data_struct.end_angle = 0x0;     //< 小数位有效个数
	ext_client_custom_graphic_line.grapic_data_struct.width = 2;
	ext_client_custom_graphic_line.grapic_data_struct.start_x = x;  
	ext_client_custom_graphic_line.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_line.grapic_data_struct.radius = 0x0;        //< 低10位
	ext_client_custom_graphic_line.grapic_data_struct.end_x =  end_x;	     //< 中11位
	ext_client_custom_graphic_line.grapic_data_struct.end_y =  end_y;			 //< 高11位
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_line, 28, 0xFFFF);
	ext_client_custom_graphic_line.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_line.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_line,30);
}

/**
  * @brief  在2号图层画圆的程序
	*/
void referee_draw_circle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t R)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_circle.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_circle.frame_header.data_length = 21;
	ext_client_custom_graphic_circle.frame_header.seq = 0;
	ext_client_custom_graphic_circle.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_circle.frame_header,4,0xFF); 
	ext_client_custom_graphic_circle.cmd_id = 0x0301;
	ext_client_custom_graphic_circle.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_circle.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_circle.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[0]=circle_index;
	ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	ext_client_custom_graphic_circle.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	ext_client_custom_graphic_circle.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_circle.grapic_data_struct.graphic_tpye = 2;
	ext_client_custom_graphic_circle.grapic_data_struct.layer = layer;
	ext_client_custom_graphic_circle.grapic_data_struct.color = color;
	ext_client_custom_graphic_circle.grapic_data_struct.start_angle = 18;
	ext_client_custom_graphic_circle.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	ext_client_custom_graphic_circle.grapic_data_struct.width = 2;
	ext_client_custom_graphic_circle.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_circle.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_circle.grapic_data_struct.radius = R;
	ext_client_custom_graphic_circle.grapic_data_struct.end_x = 0;
	ext_client_custom_graphic_circle.grapic_data_struct.end_y = 0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_circle, 28, 0xFFFF);
	ext_client_custom_graphic_circle.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_circle.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_circle,30);
}

/**
  * @brief  在2号图层画矩形的程序
	*/
void referee_draw_rectangle(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y, uint16_t end_x, uint16_t end_y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_rectangle.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_rectangle.frame_header.data_length = 21;
	ext_client_custom_graphic_rectangle.frame_header.seq = 0;
	ext_client_custom_graphic_rectangle.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_rectangle.frame_header,4,0xFF); 
	ext_client_custom_graphic_rectangle.cmd_id = 0x0301;
	ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_rectangle.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[0]=circle_index;
	ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	ext_client_custom_graphic_rectangle.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_rectangle.grapic_data_struct.graphic_tpye = 1;
	ext_client_custom_graphic_rectangle.grapic_data_struct.layer = layer;
	ext_client_custom_graphic_rectangle.grapic_data_struct.color = color;
	ext_client_custom_graphic_rectangle.grapic_data_struct.start_angle = 18;
	ext_client_custom_graphic_rectangle.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	ext_client_custom_graphic_rectangle.grapic_data_struct.width = 2;
	ext_client_custom_graphic_rectangle.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_rectangle.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_rectangle.grapic_data_struct.radius = 16;
	ext_client_custom_graphic_rectangle.grapic_data_struct.end_x = end_x;
	ext_client_custom_graphic_rectangle.grapic_data_struct.end_y = end_y;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_rectangle, 28, 0xFFFF);
	ext_client_custom_graphic_rectangle.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_rectangle.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_rectangle,30);
}

/**
  * @brief  画椭圆
	*/
void referee_draw_ellipe(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_arc.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_arc.frame_header.data_length = 21;
	ext_client_custom_graphic_arc.frame_header.seq = 0;
	ext_client_custom_graphic_arc.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_arc.frame_header,4,0xFF); 
	ext_client_custom_graphic_arc.cmd_id = 0x0301;
	ext_client_custom_graphic_arc.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_arc.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_arc.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[0]=circle_index;
	ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	ext_client_custom_graphic_arc.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	ext_client_custom_graphic_arc.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_arc.grapic_data_struct.graphic_tpye = 3;
	ext_client_custom_graphic_arc.grapic_data_struct.layer = layer;
	ext_client_custom_graphic_arc.grapic_data_struct.color = color;
	ext_client_custom_graphic_arc.grapic_data_struct.start_angle = 18;
	ext_client_custom_graphic_arc.grapic_data_struct.end_angle = 2;      								 //< 小数位有效个数
	ext_client_custom_graphic_arc.grapic_data_struct.width = 2;
	ext_client_custom_graphic_arc.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_arc.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_arc.grapic_data_struct.radius = 16;
	ext_client_custom_graphic_arc.grapic_data_struct.end_x = 0;
	ext_client_custom_graphic_arc.grapic_data_struct.end_y = 0;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_arc, 28, 0xFFFF);
	ext_client_custom_graphic_arc.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_arc.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_arc,30);
}

/**
  * @brief  画圆弧
	*/
void referee_draw_arc(uint8_t robot_id,uint8_t circle_index, uint8_t control_way,uint8_t color, uint8_t layer, uint16_t x,uint16_t y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_ellipe.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_ellipe.frame_header.data_length = 21;
	ext_client_custom_graphic_ellipe.frame_header.seq = 0;
	ext_client_custom_graphic_ellipe.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_ellipe.frame_header,4,0xFF); 
	ext_client_custom_graphic_ellipe.cmd_id = 0x0301;
	ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_ellipe.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[0]=circle_index;
	ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[1] = 0x0;  //< "A" 97
	ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_name[2]= 0x0;    //< "A"
	ext_client_custom_graphic_ellipe.grapic_data_struct.operate_tpye = control_way;
	ext_client_custom_graphic_ellipe.grapic_data_struct.graphic_tpye = 4;
	ext_client_custom_graphic_ellipe.grapic_data_struct.layer = layer;
	ext_client_custom_graphic_ellipe.grapic_data_struct.color = color;
	ext_client_custom_graphic_ellipe.grapic_data_struct.start_angle = 180;
	ext_client_custom_graphic_ellipe.grapic_data_struct.end_angle = 0;      								 //< 小数位有效个数
	ext_client_custom_graphic_ellipe.grapic_data_struct.width = 2;
	ext_client_custom_graphic_ellipe.grapic_data_struct.start_x = x;
	ext_client_custom_graphic_ellipe.grapic_data_struct.start_y = y;
	ext_client_custom_graphic_ellipe.grapic_data_struct.radius = 0;
	ext_client_custom_graphic_ellipe.grapic_data_struct.end_x = 20;
	ext_client_custom_graphic_ellipe.grapic_data_struct.end_y = 8;
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_ellipe, 28, 0xFFFF);
	ext_client_custom_graphic_ellipe.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_ellipe.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_ellipe,30);
}

/**
  * @brief  发送组合图形所需的函数
	*/
void draw_graph(graphic_data_struct_t *patterning, uint16_t index, uint8_t control_way, uint8_t graph, uint8_t layer, uint8_t color, uint8_t Sa, uint8_t Ea, uint8_t With,  uint16_t x,uint16_t y, uint8_t R, uint16_t Ex, uint16_t Ey)
{
	(*patterning).graphic_name[0]=index;
	(*patterning).graphic_name[1]=0;
	(*patterning).graphic_name[2]=0;
	(*patterning).operate_tpye=control_way;
	(*patterning).graphic_tpye=graph;
	(*patterning).layer=layer;
	(*patterning).color=color;
	(*patterning).start_angle=Sa;
	(*patterning).end_angle=Ea;
	(*patterning).width=With;
	(*patterning).start_x=x;
	(*patterning).start_y=y;
	(*patterning).radius=R;
	(*patterning).end_x=Ex;
	(*patterning).end_y=Ey;
}

/**
  * @brief  关于电容和相机图形
	*/
void referee_draw_nuc(uint8_t robot_id)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_patterning.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_patterning.frame_header.data_length =  81;
	ext_client_custom_graphic_patterning.frame_header.seq = 0;
	ext_client_custom_graphic_patterning.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_patterning.frame_header,4,0xFF); 
	ext_client_custom_graphic_patterning.cmd_id = 0x0301;
	ext_client_custom_graphic_patterning.ext_student_interactive_header_data.data_cmd_id = 0x0103;
	ext_client_custom_graphic_patterning.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_patterning.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	//备用
	draw_graph(&ext_client_custom_graphic_patterning.grapic_data_struct[0],96,ADD,LINE,2,YELLOW,0,0,1,1100,360,0,1100,365);//390,0,1100,395);//control_pm,FLOAT,2,CYAN_BLUE,0,0,2,1700,800,(pm & 0x3FF),((pm>>10) & 0x7FF), pm>>21);             //NUC相机
	draw_graph(&ext_client_custom_graphic_patterning.grapic_data_struct[1],95,ADD,LINE,2,YELLOW,0,0,1,1060,420,0,1060,425);//440,0,1060,445);//control_ym,FLOAT,2,CYAN_BLUE,0,0,2,1700,900,(ym & 0x3FF),((ym>>10) & 0x7FF), ym>>21);    //相机
	draw_graph(&ext_client_custom_graphic_patterning.grapic_data_struct[2],94,ADD,INT,2,YELLOW,8,0,2,(600+(20-LowVolt)*300.0/(HighVolt-LowVolt)),90,20,0,0);  //电容20标志位
	draw_graph(&ext_client_custom_graphic_patterning.grapic_data_struct[3],93,ADD,INT,2,YELLOW,8,0,2,(600+(22-LowVolt)*300.0/(HighVolt-LowVolt)),90,22,0,0);          //电容22标志位
	draw_graph(&ext_client_custom_graphic_patterning.grapic_data_struct[4],92,ADD,INT,2,YELLOW,8,0,2,(600+(24-LowVolt)*300.0/(HighVolt-LowVolt)),90,24,0,0);       //电容24标志位
 

	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_patterning, 88, 0xFFFF);
	ext_client_custom_graphic_patterning.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_patterning.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&ext_client_custom_graphic_patterning,sizeof(ext_client_custom_graphic_patterning));
	
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_patterning,90);
}


/**
  * @brief  关于电容的图形
	*/
void referee_draw_cap(uint8_t robot_id, uint8_t control_P, uint8_t DATA, uint8_t control_cap_lenghth, float VOLT,uint8_t color_cap, float error_x, float error_y)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_patterning_second.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_patterning_second.frame_header.data_length =  81;
	ext_client_custom_graphic_patterning_second.frame_header.seq = 0;
	ext_client_custom_graphic_patterning_second.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_patterning_second.frame_header,4,0xFF); 
	ext_client_custom_graphic_patterning_second.cmd_id = 0x0301;
	ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.data_cmd_id = 0x0103;
	ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_patterning_second.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	
		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[0],91,control_P,INT,2,GREEN,18,0,2,200,550,DATA,0,0);   //电容充电标志位

		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[1],90,ADD,LINE,2,YELLOW,0,0,1,752,330,0,1168,330); 	//准线
	
		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[2],89,ADD,RECTANGLE,2,YELLOW,0,0,2,600,100,0,900,130);  //电容框
	if(VOLT<50)
		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[3],88,control_cap_lenghth,LINE,2,color_cap,0,0,30,600,115,0,(600+(VOLT-LowVolt)*300.0/(HighVolt-LowVolt)),115);     //电容百分比
		
	if(error_x<1&&error_y<1)
		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[4],87,ADD,CIRCLE,2,CYAN_BLUE,18,2,2,960,540,8,0,0);    //瞄准出现圆
	else 
		draw_graph(&ext_client_custom_graphic_patterning_second.grapic_data_struct[4],87,DELETE,CIRCLE,2,CYAN_BLUE,18,2,2,960,540,8,0,0);    //瞄准出现圆
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_patterning_second, 88, 0xFFFF);
	ext_client_custom_graphic_patterning_second.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_patterning_second.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&ext_client_custom_graphic_patterning_second,sizeof(ext_client_custom_graphic_patterning_second));
	
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_patterning_second,90);
}

/**
  * @brief  关于底盘的图形
	*/
void referee_draw_chassis(uint8_t robot_id, uint8_t control_rotation, uint8_t control_follow, uint8_t control_boom, uint8_t control_shoot,uint8_t rotation,uint8_t follow,uint8_t boom,uint8_t shoot)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_patterning_third.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_patterning_third.frame_header.data_length =  81;
	ext_client_custom_graphic_patterning_third.frame_header.seq = 0;
	ext_client_custom_graphic_patterning_third.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_patterning_third.frame_header,4,0xFF); 
	ext_client_custom_graphic_patterning_third.cmd_id = 0x0301;
	ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.data_cmd_id = 0x0103;
	ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_patterning_third.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	if(rotation==0)
		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[0],84,control_rotation,INT,3,YELLOW,18,0,2,200,650,0,0,0);   //自旋
	else if(rotation==1)
		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[0],84,control_rotation,INT,3,YELLOW,18,0,2,200,650,1,0,0);   //自旋
	if(follow==0)
		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[1],83,control_follow,INT,3,RED_BLUE,18,0,2,200,700,0,0,0);  //跟随
	else if(follow==1)
		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[1],83,control_follow,INT,3,RED_BLUE,18,0,2,200,700,1,0,0);  //跟随
//	if(boom==0)
//		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[2],82,control_boom,INT,3,RED_BLUE,18,0,2,200,800,0,0,0);  //自爆
//	else if(boom==1)
//		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[2],82,control_boom,INT,3,RED_BLUE,18,0,2,200,800,1,0,0);  //自爆
//	if(shoot==0)
//		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[3],81,control_shoot,INT,3,FUCHSIA,18,0,2,200,600,0,0,0);     //吊射
//	else if(shoot==1)
//		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[3],81,control_shoot,INT,3,FUCHSIA,18,0,2,200,600,1,0,0);     //吊射
	
		draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[4],80,ADD,INT,3,YELLOW,8,0,2,(600+(16-LowVolt)*300.0/(HighVolt-LowVolt)),90,16,0,0);     //电容16v标志
	
//	draw_graph(&ext_client_custom_graphic_patterning_third.grapic_data_struct[4],80,control_head,INT,3,GREEN,0,0,2,1770,540,0,-20,8);     //转头//
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_patterning_third, 88, 0xFFFF);
	ext_client_custom_graphic_patterning_third.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_patterning_third.CRC16[1] = crc16_temp >> 8;
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_patterning_third,90);
}

/**
  * @brief  关于准星的图形
	*/
void referee_draw_shoot(uint8_t robot_id)
{
	uint16_t crc16_temp;
	ext_client_custom_graphic_patterning_fourth.frame_header.SOF = 0xA5;
	ext_client_custom_graphic_patterning_fourth.frame_header.data_length =  111;
	ext_client_custom_graphic_patterning_fourth.frame_header.seq = 0;
	ext_client_custom_graphic_patterning_fourth.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_patterning_fourth.frame_header,4,0xFF); 
	ext_client_custom_graphic_patterning_fourth.cmd_id = 0x0301;
	ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.data_cmd_id = 0x0104;
	ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.sender_ID = robot_id;
	ext_client_custom_graphic_patterning_fourth.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
	
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[0],78,ADD,LINE,2,YELLOW,0,0,1,970,542,0,970,547);//395);//control_p,INT,2,YELLOW,0,0,2,1700,700,(DATA1 & 0x3FF),((DATA1>>10) & 0x7FF),(DATA1>>21));      //车间线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[1],77,ADD,LINE,2,YELLOW,0,0,1,950,542,0,950,547);//395);//control_y,INT,2,CYAN_BLUE,0,0,2,1700,600,(DATA2 & 0x3FF),((DATA2>>10) & 0x7FF),(DATA2>>21));      //车间线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[2],76,ADD,LINE,2,YELLOW,0,0,1,802,360,0,1108,360);//390,0,1132,390);//540,0,1132,540);      //车间线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[3],75,ADD,LINE,2,YELLOW,0,0,1,960,800,0,960,200);  	 	 //准线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[4],74,ADD,LINE,2,YELLOW,0,0,1,938,540,0,982,540);//390,0,982,390);      //准线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[5],73,ADD,LINE,2,YELLOW,0,0,1,852,420,0,1068,420);//440,0,1072,440);//490,0,1072,490);			 //准线
	draw_graph(&ext_client_custom_graphic_patterning_fourth.grapic_data_struct[6],72,ADD,LINE,2,YELLOW,0,0,1,902,480,0,1032,480);//490,0,1022,490);//440,0,1022,440);      //准线
	
	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_patterning_fourth, 118, 0xFFFF);
	ext_client_custom_graphic_patterning_fourth.CRC16[0] = crc16_temp & 0xFF;
	ext_client_custom_graphic_patterning_fourth.CRC16[1] = crc16_temp >> 8;
	
	memcpy(Data_Pack,(unsigned char*)&ext_client_custom_graphic_patterning_fourth,sizeof(ext_client_custom_graphic_patterning_fourth));
	
	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_patterning_fourth,120);
}




///**
//  * @brief  关于准星的图形
//	*/
//void referee_draw_shoot_two(uint8_t robot_id)
//{
//	uint16_t crc16_temp;
//	ext_client_custom_graphic_patterning_fifth.frame_header.SOF = 0xA5;
//	ext_client_custom_graphic_patterning_fifth.frame_header.data_length =  111;
//	ext_client_custom_graphic_patterning_fifth.frame_header.seq = 0;
//	ext_client_custom_graphic_patterning_fifth.frame_header.CRC8 = Get_CRC8_Check((unsigned char*)&ext_client_custom_graphic_patterning_fifth.frame_header,4,0xFF); 
//	ext_client_custom_graphic_patterning_fifth.cmd_id = 0x0301;
//	ext_client_custom_graphic_patterning_fifth.ext_student_interactive_header_data.data_cmd_id = 0x0104;
//	ext_client_custom_graphic_patterning_fifth.ext_student_interactive_header_data.sender_ID = robot_id;
//	ext_client_custom_graphic_patterning_fifth.ext_student_interactive_header_data.receiver_ID = referee_get_receiver_ID(robot_id);
//	
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[0],78,ADD,LINE,2,YELLOW,0,0,1,970,542,0,970,547);//395);//control_p,INT,2,YELLOW,0,0,2,1700,700,(DATA1 & 0x3FF),((DATA1>>10) & 0x7FF),(DATA1>>21));      //车间线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[1],77,ADD,LINE,2,YELLOW,0,0,1,950,542,0,950,547);//395);//control_y,INT,2,CYAN_BLUE,0,0,2,1700,600,(DATA2 & 0x3FF),((DATA2>>10) & 0x7FF),(DATA2>>21));      //车间线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[2],76,ADD,LINE,2,YELLOW,0,0,1,802,360,0,1108,360);//390,0,1132,390);//540,0,1132,540);      //车间线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[3],75,ADD,LINE,2,YELLOW,0,0,1,960,800,0,960,200);  	 	 //准线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[4],74,ADD,LINE,2,YELLOW,0,0,1,938,540,0,982,540);//390,0,982,390);      //准线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[5],73,ADD,LINE,2,YELLOW,0,0,1,852,420,0,1068,420);//440,0,1072,440);//490,0,1072,490);			 //准线
//	draw_graph(&ext_client_custom_graphic_patterning_fifth.grapic_data_struct[6],72,ADD,LINE,2,YELLOW,0,0,1,902,480,0,1032,480);//490,0,1022,490);//440,0,1022,440);      //准线
//	
//	crc16_temp = Get_CRC16_Check((unsigned char*)&ext_client_custom_graphic_patterning_fifth, 118, 0xFFFF);
//	ext_client_custom_graphic_patterning_fifth.CRC16[0] = crc16_temp & 0xFF;
//	ext_client_custom_graphic_patterning_fifth.CRC16[1] = crc16_temp >> 8;
//	
//	memcpy(Data_Pack,(unsigned char*)&ext_client_custom_graphic_patterning_fifth,sizeof(ext_client_custom_graphic_patterning_fifth));
//	
//	HAL_UART_Transmit_DMA(&huart3,(unsigned char*)&ext_client_custom_graphic_patterning_fifth,120);
//}

/**
  * @brief  画NUC
	*/
void referee_draw_NUC_data(uint32_t cnt , uint8_t robot_id, uint16_t mode)
{
	if(cnt%200==0 && cnt<3000)
	{
	 referee_draw_nuc(robot_id);
	}
	
//	if(cnt%100==0 && cnt>3000)
//	{
//	 referee_draw_nuc(robot_id,MODIFY,MODIFY,pm_data,ym_data);
//	}
//	
//	if(cnt%100 == 10 && (cnt > 1000))
//	{
//		if(mode==1)        																													 //击打1号机器人
//			referee_draw_int(robot_id, SHOOTONE, 86, MODIFY, RED_BLUE, 550, 135);   
//		else if(mode==2)																													  	 //击打2号机器人
//			referee_draw_int(robot_id, SHOOTTWO, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==3) 																														//击打3号机器人
//			referee_draw_int(robot_id, SHOOTTHREE, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==4)																															//击打4号机器人
//			referee_draw_int(robot_id, SHOOTFOUR, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==5)																															//击打5号机器人
//			referee_draw_int(robot_id, SHOOTFIVE, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==6)																															//击打哨兵机器人
//			referee_draw_int(robot_id, SHOOTSENTRY, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==7)																															//射击前哨战
//			referee_draw_int(robot_id, SHOOTOUTPOST, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==8)																															//射击基地 
//			referee_draw_int(robot_id, SHOOTBASE, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==10)																															//吊射前哨战
//			referee_draw_int(robot_id, CURVEDFIREOUTPOST, 86, MODIFY, RED_BLUE, 550, 135);
//		else if(mode==11)																															//吊射基地
//			referee_draw_int(robot_id, CURVEDFIREBASE, 86, MODIFY, RED_BLUE, 550, 135);
//		else 
//			referee_draw_int(robot_id, 0, 86, ADD, RED_BLUE, 550, 135);
//	}
}



/**
  * @brief  画发射机构
	*/
void referee_draw_Load_data(uint32_t cnt , uint8_t robot_id,uint8_t rotation,uint8_t follow,uint8_t boom,uint8_t shoot)
{	
	if(cnt%100 == 40 && (cnt < 3000)){
		referee_draw_chassis(robot_id,ADD,ADD,ADD,ADD,rotation,follow,boom,shoot);
	}
	
	
	if(cnt%100 == 40 && (cnt > 3000)){																													
			referee_draw_chassis(robot_id, MODIFY, MODIFY, MODIFY, MODIFY,rotation, follow, boom, shoot);
	}
	
}


/**
  * @brief  画底盘
	*/
void referee_draw_chassis_data(uint32_t cnt , uint8_t robot_id, float pitch_angle_mpu, float yaw_angle_mpu, float pitch_angle, float yaw_angle)
{	
	if(cnt%100==0 && cnt<2000)
		referee_draw_float(robot_id,pitch_angle,65, ADD, YELLOW,0, 1600, 600);
	if(cnt%100==10 && cnt<2000)
		referee_draw_float(robot_id,yaw_angle,66, ADD, YELLOW, 0,1600, 500);
  if(cnt%100==20 && cnt<2000)
		referee_draw_float(robot_id, pitch_angle_mpu,67, ADD,YELLOW, 0, 1600,650);
  if(cnt%100==50 && cnt<2000)
		referee_draw_float(robot_id, yaw_angle_mpu,68, ADD,YELLOW, 0, 1600,550);
	
	if(cnt%100==0 && cnt>2000)
		referee_draw_float(robot_id,pitch_angle,65, MODIFY, YELLOW,0, 1600, 600);
	if(cnt%100==10 && cnt>2000)
		referee_draw_float(robot_id,yaw_angle,66, MODIFY, YELLOW, 0,1600, 500);
  if(cnt%100==20 && cnt>2000)
		referee_draw_float(robot_id, pitch_angle_mpu,67, MODIFY,YELLOW, 0, 1600,650);
  if(cnt%100==50 && cnt>2000)
		referee_draw_float(robot_id, yaw_angle_mpu,68, MODIFY,YELLOW, 0, 1600,550);

}


/**
  * @brief  画电容
	*/
void referee_draw_supercap_data(uint32_t cnt, uint8_t robot_id, float capvolt, float error_x, float error_y,uint8_t DATA)
{
	if(cnt%100 == 30 && (cnt < 2000))
	{
		referee_draw_cap(robot_id, ADD, DATA, ADD, capvolt, YELLOW, error_x, error_y);
	}
	
	if(cnt%100 == 30 && (cnt > 2000)){
		if(capvolt<17)
			referee_draw_cap(robot_id, MODIFY, DATA, MODIFY, capvolt, RED_BLUE, error_x, error_y);
		else 
			referee_draw_cap(robot_id, MODIFY, DATA, MODIFY, capvolt, YELLOW, error_x, error_y);
	}
}

void referee_draw_shoot_data(uint32_t cnt , uint8_t robot_id)
{
	char *K1="L   L    L    L   L    L   L   L    L    L   L ";
	char *K2="L     L     L     L     L     L ";
	char *K3=" L     L     L    ";
	
	if(cnt%100==90)
		referee_draw_shoot(robot_id);
	
//	if(cnt%100-==90)
//		referee_draw_shoot_two(robot_id);
	
	if(cnt%100==60)
	referee_draw_string(robot_id, K1,59,ADD,YELLOW,8, 7, 835,367);//397);

	if(cnt%100==70)
	referee_draw_string(robot_id, K2,58,ADD,YELLOW,8, 7, 855,427);//447);//497);

	if(cnt%100==80)
	referee_draw_string(robot_id, K3,57,ADD,YELLOW,8, 7, 910,487);//497);//447);
	
//	if(cnt%100==60)
//	referee_draw_string(robot_id, K1,59,ADD,YELLOW,8, 7, 835,547);

//	if(cnt%100==70)
//	referee_draw_string(robot_id, K2,58,ADD,YELLOW,8, 7, 855,497);

//	if(cnt%100==80)
//	referee_draw_string(robot_id, K3,57,ADD,YELLOW,8, 7, 910,447);

}



/**
  * @brief  关于刷新后图形
	*/
void update(uint32_t cnt, uint8_t robot_id, uint8_t change)
{
	if(change)
	{
		referee_graphic_delete(0, ALL_delete, robot_id);
		cnt=0;
	}
}



uint16_t referee_get_receiver_ID(uint32_t sender_id)
{
	switch(sender_id)
	{
		case RobotRedHero: return ClientRedHero; 
		case RobotRedEngineer: return ClientRedEngineer;
		case RobotRedInfantryNO3: return ClientRedInfantryNO3;
		case RobotRedInfantryNO4: return ClientRedInfantryNO4; 
		case RobotRedInfantryNO5: return ClientRedInfantryNO5; 
		case RobotRedAerial: return ClientRedAerial;
		case RobotBlueHero: return ClientBlueHero;
		case RobotBlueEngineer: return ClientBlueEngineer; 
		case RobotBlueInfantryNO3: return ClientBlueInfantryNO3;
		case RobotBlueInfantryNO4: return ClientBlueInfantryNO4; 
		case RobotBlueInfantryNO5: return ClientBlueInfantryNO5; 
		case RobotBlueAerial: return ClientBlueAerial; 
		default: return 0;
	}
}




const unsigned char CRC8__INIT = 0xff;
const unsigned char CRC8__TAB[256] =//8位最多256个
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned char Get_CRC8_Check(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;                     //与0异或保持不变，与1异或反转
	while (dwLength--)   //ucCRC8是什么??
	{
		ucIndex = ucCRC8^(*pchMessage++);//第一次是取反?? 
		ucCRC8 = CRC8__TAB[ucIndex];//余式表             
	}                                                   
	return(ucCRC8);
}

uint16_t CRC__INIT = 0xffff;
const uint16_t wCRC__Table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)//无效地址
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC__Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
	}
	return wCRC;
}



