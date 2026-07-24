#include "dr16.h"




//	RC_Ctrl rc_Ctrl={
//		.isUnpackaging = 0,
//		.isOnline = 0	
//	};


///**
//		* @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
//		*/		

//void DR16Init(RC_Ctrl* RC_Ctl)
//{
//		RC_Ctl->rc.ch0=1024;
//		RC_Ctl->rc.ch1=1024;
//		RC_Ctl->rc.ch2=1024;
//		RC_Ctl->rc.ch3=1024;
//		RC_Ctl->rc.s1=3;
//		RC_Ctl->rc.s2=3;
//		RC_Ctl->rc.sw=1024;
//		RC_Ctl->mouse.x=0;
//		RC_Ctl->mouse.y=0;
//		RC_Ctl->mouse.z=0;		
//		RC_Ctl->a = 0;
//		RC_Ctl->key_Q_flag=0;
//		RC_Ctl->key_E_flag=0;    
//		RC_Ctl->key_R_flag=0;
//		RC_Ctl->key_F_flag=0;
//		RC_Ctl->key_G_flag=0;
//		RC_Ctl->key_Z_flag=0;
//		RC_Ctl->key_X_flag=0;
//		RC_Ctl->key_C_flag=0;
//		RC_Ctl->key_V_flag=0;
//		RC_Ctl->key_B_flag=0;
//		RC_Ctl->Chassis_Y_Integ=0;//斜坡积分变量
//		RC_Ctl->Chassis_X_Integ=0;
//}
//	
///**
//   * @brief  创建dr16的接收机缓存数组, 并对全局变量rc_Ctrl赋值，以供其他函数调用
//   */
//void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer)
//{
//		rc_ctrl->isUnpackaging = 1;					//< 解算期间不允许读取数据
//		uint8_t correct_num=0;	
//		correct_num=0;
//		if(((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)<=1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)>=364)
//			correct_num++;
//		if((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)<=1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)>=364)
//			correct_num++;
//		if((((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)<=1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)>=364)
//			correct_num++;
//		if((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)<=1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)>=364)
//			correct_num++;
//		if((((recBuffer[5] >> 4)& 0x000C) >> 2)==1 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==2 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==3)
//			correct_num++;
//		if(((recBuffer[5] >> 4)& 0x0003)==1 || ((recBuffer[5] >> 4)& 0x0003)==2 || ((recBuffer[5] >> 4)& 0x0003)==3)
//			correct_num++;
//		if(correct_num==6)																																												//< 数据完整性验证 
//		{  	
//			rc_ctrl->rc.ch0 = (recBuffer[0]| (recBuffer[1] << 8)) & 0x07ff; 																					//< Channel 0   高8位与低3位
//			rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff; 																	//< Channel 1   高5位与低6位
//			rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff; 						//< Channel 2
//			rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff; 																	//< Channel 3
//			rc_ctrl->rc.s1 = ((recBuffer[5] >> 4)& 0x000C) >> 2; 																											//!< Switch left
//			rc_ctrl->rc.s2 = ((recBuffer[5] >> 4)& 0x0003); 																													//!< Switch right
//			rc_ctrl->rc.sw=(uint16_t)(recBuffer[16]|(recBuffer[17]<<8))&0x7ff; 
//				
//				/***********按键映射*************/
//			rc_ctrl->mouse.x = recBuffer[6]  | (recBuffer[7] << 8);                       //< Mouse X axis   
//			rc_ctrl->mouse.y = recBuffer[8]  | (recBuffer[9] << 8);                       //< Mouse Y axis     
//			rc_ctrl->mouse.z = recBuffer[10] | (recBuffer[11] << 8);                      //< Mouse Z axis   
//			rc_ctrl->mouse.press_l = recBuffer[12];                                       //< Mouse Left Is Press ?   
//			rc_ctrl->mouse.press_r = recBuffer[13];                                       //< Mouse Right Is Press ? 
//							
//			if(rc_ctrl->mouse.x>25000)   rc_ctrl->mouse.x=25000;     																												//< 限幅
//			if(rc_ctrl->mouse.x<-25000)  rc_ctrl->mouse.x=-25000;	
//			if(rc_ctrl->mouse.y>25000)   rc_ctrl->mouse.y=25000;
//			if(rc_ctrl->mouse.y<-25000)  rc_ctrl->mouse.y=-25000;
//				
//			rc_ctrl->keyboard.v = recBuffer[14]| (recBuffer[15] << 8);  									//< 共16个按键值   

//			rc_ctrl->key_W=recBuffer[14]&0x01;	
//			rc_ctrl->key_S=(recBuffer[14]>>1)&0x01;					
//			rc_ctrl->key_A=(recBuffer[14]>>2)&0x01;
//			rc_ctrl->key_D=(recBuffer[14]>>3)&0x01;					
//			rc_ctrl->key_B=(recBuffer[15]>>7)&0x01;
//			rc_ctrl->key_V=(recBuffer[15]>>6)&0x01;				
//			rc_ctrl->key_C=(recBuffer[15]>>5)&0x01;
//			rc_ctrl->key_X=(recBuffer[15]>>4)&0x01;					
//			rc_ctrl->key_Z=(recBuffer[15]>>3)&0x01;					
//			rc_ctrl->key_G=(recBuffer[15]>>2)&0x01;			
//			rc_ctrl->key_F=(recBuffer[15]>>1)&0x01;
//			rc_ctrl->key_R=(recBuffer[15])&0x01;													
//			rc_ctrl->key_E=(recBuffer[14]>>7)&0x01;
//			rc_ctrl->key_Q=(recBuffer[14]>>6)&0x01;
//			rc_ctrl->key_ctrl=(recBuffer[14]>>5)&0x01;
//			rc_ctrl->key_shift=(recBuffer[14]>>4)&0x01;
//			PC_keybroad_filter(rc_ctrl);				//< 防抖

//					}
//		else{}
//		rc_ctrl->isUnpackaging = 0;					//< 解算完成标志位，允许读取
//}




//	/**
//		* @brief  按键消抖，检测是否为有效按下
//		*/
//	void PC_keybroad_filter(RC_Ctrl* RC_Ctl)
//	{
//		static uint16_t key_W_cnt,key_A_cnt,key_S_cnt,key_D_cnt,key_ctrl_cnt,
//									 key_shift_cnt,mouse_press_l_cnt,mouse_press_r_cnt,
//									 key_C_cnt,key_F_cnt,key_G_cnt,key_Q_cnt,key_E_cnt,
//										key_Z_cnt,key_V_cnt,key_X_cnt,key_B_cnt,key_R_cnt;
//			/*   支持连续按 W A S D   */
///*-------------------------------Key W--------------------------------*/
//		if(RC_Ctl->key_W==1) 
//		{
//			key_W_cnt++;
//			if(key_W_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_W_flag=1;	
//			}	 
//		}   
//		else
//		{
//			RC_Ctl->key_W_flag=0;	
//			key_W_cnt=0;	
//		}	
///*-------------------------------Key A--------------------------------*/
//		if(RC_Ctl->key_A==1) 
//		{
//			key_A_cnt++;
//			if(key_A_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_A_flag=1;
//			}	
//		}

//		else
//		{		
//			RC_Ctl->key_A_flag=0;
//			key_A_cnt=0;
//		}
//		
///*-------------------------------Key S--------------------------------*/
//		if(RC_Ctl->key_S==1) 
//		{
//			key_S_cnt++;
//			if(key_S_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_S_flag=1;	
//			}			
//		}	
//		else
//		{
//			RC_Ctl->key_S_flag=0;
//			key_S_cnt=0;
//		}	
//		
///*-------------------------------Key D--------------------------------*/
//		if(RC_Ctl->key_D==1) 
//		{
//			key_D_cnt++;
//			if(key_D_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_D_flag=1;			
//			}	
//		}		
//		else
//		{
//			RC_Ctl->key_D_flag=0;
//			key_D_cnt=0;
//		}
//		
///*-------------------------------Key B--------------------------------*/
//		if(RC_Ctl->key_B==1) 
//		{
//			key_B_cnt++;
//			if(key_B_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_B_flag=1;
//			}	
//		}	
//		else
//		{
//			RC_Ctl->key_B_flag=0;
//			key_B_cnt=0;
//		}
//		
///*-------------------------------Key C--------------------------------*/
//		if(RC_Ctl->key_C==1) 
//		{
//			key_C_cnt++;
//			if(key_C_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_C_flag=1;
//			}	 
//		}  
//		else
//		{
//			RC_Ctl->key_C_flag=0;
//			key_C_cnt=0;
//		}	
//		
///*-------------------------------Key R--------------------------------*/
//		if(RC_Ctl->key_R==1) 
//		{
//			key_R_cnt++;
//			if(key_R_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_R_flag=1;			
//			}	 
//		}  
//		else
//		 {
//	   	RC_Ctl->key_R_flag=0;
//			key_R_cnt=0;	
//		 }
//		 
///*-------------------------------Key F--------------------------------*/
//		if(RC_Ctl->key_F==1) 
//		{
//			key_F_cnt++;
//			if(key_F_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_F_flag =1;
//			}	
//		}	
//		else
//		{
//  		RC_Ctl->key_F_flag=0;
//			key_F_cnt=0;
//		}
//		
//		
///*-------------------------------Key X--------------------------------*/
//		if(RC_Ctl->key_X==1) 
//		{
//			key_X_cnt++;
//			if(key_X_cnt==Key_Filter_Num)	
//			{
//			RC_Ctl->key_X_flag=1;			
//			}	 
//		}	
//		else
//		{
//			RC_Ctl->key_X_flag=0;
//			key_X_cnt=0;
//		}
//		
//		/*-------------------------------Key G--------------------------------*/
//		if(RC_Ctl->key_G==1) 
//		{
//			key_G_cnt++;
//			if(key_G_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_G_flag=1;
//			}	
//		}	
//		else
//		{
//			RC_Ctl->key_G_flag=0;
//			key_G_cnt=0;
//		}	

//		
///*-------------------------------Key Q--------------------------------*/
//		if(RC_Ctl->key_Q==1) 
//		{
//			key_Q_cnt++;
//			if(key_Q_cnt==Key_Filter_Num)	
//			{
//			RC_Ctl->key_Q_flag=1;			
//			}	
//		}	
//		else
//		{
//			RC_Ctl->key_Q_flag=0;
//			key_Q_cnt=0;
//		}
//		
//		
///*-------------------------------Key E--------------------------------*/
//		if(RC_Ctl->key_E==1) 
//		 {
//			key_E_cnt++;
//			if(key_E_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_E_flag=1;	
//		  }
//	 }
//		 else
//		 {
//			RC_Ctl->key_E_flag=0;	 
//			key_E_cnt=0;
//		 }	
//		 
///*-------------------------------Key Z--------------------------------*/
//			if(RC_Ctl->key_Z==1) 
//		{
//			key_Z_cnt++;
//			if(key_Z_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_Z_flag=1;
//			}			
//		}	
//		else
//		 {
//			RC_Ctl->key_Z_flag=0;
//			key_Z_cnt=0;
//		 } 
//		 
///*-------------------------------Key V--------------------------------*/
//			if(RC_Ctl->key_V==1) 
//		{
//			key_V_cnt++;
//			if(key_V_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_V_flag=1;		
//			}			
//		}	
//		else
//		 {
//			RC_Ctl->key_V_flag=0;
//			key_V_cnt=0;	
//		 } 	 
//		 
///*-------------------------------Key ctrl--------------------------------*/
//		if(RC_Ctl->key_ctrl==1) 
//		{
//			key_ctrl_cnt++;
//			if(key_ctrl_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_ctrl_flag=1;
//			}	
//		} 
//		else
//		{
//			RC_Ctl->key_ctrl_flag=0;
//			key_ctrl_cnt=0;	
//		}	 
///*-------------------------------Key shift--------------------------------*/
//		 if(RC_Ctl->key_shift==1) 
//		{
//			key_shift_cnt++;
//			if(key_shift_cnt==Key_Filter_Num)	
//			{
//				RC_Ctl->key_shift_flag=1;
//			}	
//		 }	
//		else
//		{
//			RC_Ctl->key_shift_flag=0;
//			key_shift_cnt=0;	
//		}
//		 //mouse_l
///*-------------------------------Key mouse_l--------------------------------*/
//		 if(RC_Ctl->mouse.press_l==1)
//		 {
//			mouse_press_l_cnt++;
//			if(mouse_press_l_cnt==Key_Filter_Num)
//			{
//				RC_Ctl->mouse.press_l_flag=1;
//			}
//		 }
//		 else
//		 {
//			 RC_Ctl->mouse.press_l_flag=0;
//			 mouse_press_l_cnt=0;
//		 }
//			
///*-------------------------------Key mouse_r--------------------------------*/

//		 if(RC_Ctl->mouse.press_r==1)
//		 {
//			mouse_press_r_cnt++;
//			if(mouse_press_r_cnt==Key_Filter_Num)
//			{
//				RC_Ctl->mouse.press_r_flag=1;
//			}
//		 }
//		 else
//		 {
//			 
//			 RC_Ctl->mouse.press_r_flag=0;
//				mouse_press_r_cnt=0;
//		 }
//		 
//		 
//	}





