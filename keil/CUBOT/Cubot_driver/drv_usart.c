/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  涓插彛bsp灞傜殑瀹炵幇
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "drv_usart.h"
#include "drv_log.h"
#include "stdlib.h"
//#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart鏈嶅姟瀹炰緥,鎵€鏈夋敞鍐屼簡usart鐨勬ā鍧椾俊鎭細琚繚瀛樺湪杩欓噷 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 鍚姩涓插彛鏈嶅姟,浼氬湪姣忎釜瀹炰緥娉ㄥ唽涔嬪悗鑷姩鍚敤鎺ユ敹,褰撳墠瀹炵幇涓篋MA鎺ユ敹,鍚庣画鍙兘娣诲姞IT鍜孊LOCKING鎺ユ敹
 *
 * @todo 涓插彛鏈嶅姟浼氬湪姣忎釜瀹炰緥娉ㄥ唽涔嬪悗鑷姩鍚敤鎺ユ敹,褰撳墠瀹炵幇涓篋MA鎺ユ敹,鍚庣画鍙兘娣诲姞IT鍜孊LOCKING鎺ユ敹
 *       鍙兘杩樿灏嗘鍑芥暟淇敼涓篹xtern,浣垮緱module鍙互鎺у埗涓插彛鐨勫惎鍋?
 *
 * @param _instance instance owned by module,妯″潡鎷ユ湁鐨勪覆鍙ｅ疄渚?
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 鍏抽棴dma half transfer涓柇闃叉涓ゆ杩涘叆HAL_UARTEx_RxEventCallback()
    // 杩欐槸HAL搴撶殑涓€涓璁″け璇?鍙戠敓DMA浼犺緭瀹屾垚/鍗婂畬鎴愪互鍙婁覆鍙DLE涓柇閮戒細瑙﹀彂HAL_UARTEx_RxEventCallback()
    // 鎴戜滑鍙笇鏈涘鐞嗙涓€绉嶅拰绗笁绉嶆儏鍐?鍥犳鐩存帴鍏抽棴DMA鍗婁紶杈撲腑鏂?
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT)
	{ // 瓒呰繃鏈€澶у疄渚嬫暟
//        while (1)
//				{
//        //    LOGERROR("[bsp_usart] USART exceed max instance count!");
//				}
			}

    for (uint8_t i = 0; i < idx; i++) 
	{
			// 妫€鏌ユ槸鍚﹀凡缁忔敞鍐岃繃
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
				{
//            while (1)
//						{
//							//LOGERROR("[bsp_usart] USART instance already registered!");
//							
//						}
				}
    }
    
    // 1. 鐢宠缁撴瀯浣撳唴瀛?
    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));
    
    // 2. 璧嬪€?
    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    
    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 褰撳墠浠呰繘琛屼簡褰㈠紡涓婄殑灏佽,鍚庣画瑕佽繘涓€姝ヨ€冭檻鏄惁灏唌odule鐨勮涓轰笌bsp瀹屽叏鍒嗙 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 妫€鏌ュ畾涔塱nstance鐨勪唬鐮佷笂涓嬫枃,鍙兘鍑虹幇鎸囬拡瓒婄晫
        break;
    }
}

/* 涓插彛鍙戦€佹椂,gstate浼氳璁句负BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    return (_instance != NULL &&
            _instance->usart_handle != NULL &&
            _instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1u : 0u;
}

/**
 * @brief 姣忔dma/idle涓柇鍙戠敓鏃讹紝閮戒細璋冪敤姝ゅ嚱鏁?瀵逛簬姣忎釜uart瀹炰緥浼氳皟鐢ㄥ搴旂殑鍥炶皟杩涜杩涗竴姝ョ殑澶勭悊
 *        渚嬪:瑙嗚鍗忚瑙ｆ瀽/閬ユ帶鍣ㄨВ鏋?瑁佸垽绯荤粺瑙ｆ瀽
 *
 * @note  閫氳繃__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)鍏抽棴dma half transfer涓柇闃叉涓ゆ杩涘叆HAL_UARTEx_RxEventCallback()
 *        杩欐槸HAL搴撶殑涓€涓璁″け璇?鍙戠敓DMA浼犺緭瀹屾垚/鍗婂畬鎴愪互鍙婁覆鍙DLE涓柇閮戒細瑙﹀彂HAL_UARTEx_RxEventCallback()
 *        鎴戜滑鍙笇鏈涘鐞嗭紝鍥犳鐩存帴鍏抽棴DMA鍗婁紶杈撲腑鏂涓€绉嶅拰绗笁绉嶆儏鍐?
 *
 * @param huart 鍙戠敓涓柇鐨勪覆鍙?
 * @param Size 姝ゆ鎺ユ敹鍒扮殑鎬绘暟灞呴噺,鏆傛椂娌＄敤
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            /*
             * ReceiveToIdle 的 Size 可能小于配置缓冲区长度。必须把实际长度
             * 交给协议层，不能默认每次回调都得到一整帧。
             */
            usart_instance[i]->recv_data_len = Size;
            // 妯″潡鍥炶皟
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 鎺ユ敹缁撴潫鍚庢竻绌篵uffer,瀵逛簬鍙橀暱鏁版嵁鏄繀瑕佺殑
            }
            // 鍒濆鍖栧苟鍚姩 鈥淒MA + 绌洪棽涓柇鈥?妯″紡鐨勪覆鍙ｆ帴鏀?-> DMA 鎺у埗鍣ㄤ細鑷姩鎶?RDR 鐨勫瓧鑺傛惉鍒板唴瀛?recv_buff[]
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            // 绂佺敤杩欎釜 DMA 鐨勫崐浼犺緭涓柇
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 褰撲覆鍙ｅ彂閫?鎺ユ敹鍑虹幇閿欒鏃?浼氳皟鐢ㄦ鍑芥暟,姝ゆ椂杩欎釜鍑芥暟瑕佸仛鐨勫氨鏄噸鏂板惎鍔ㄦ帴鏀?
 *
 * @note  鏈€甯歌鐨勯敊璇?濂囧伓鏍￠獙/婧㈠嚭/甯ч敊璇?
 *
 * @param huart 鍙戠敓閿欒鐨勪覆鍙?
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            //  LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}

// 璋冭瘯涓嶅彈鎺у埗
///**
// * @brief 褰撲覆鍙ｅ彂閫?鎺ユ敹鍑虹幇閿欒鏃?浼氳皟鐢ㄦ鍑芥暟,姝ゆ椂杩欎釜鍑芥暟瑕佸仛鐨勫氨鏄噸鏂板惎鍔ㄦ帴鏀?
// *
// * @note  鏈€甯歌鐨勯敊璇?濂囧伓鏍￠獙/婧㈠嚭/甯ч敊璇?
// *
// * @param huart 鍙戠敓閿欒鐨勪覆鍙?
// */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    // ============================================================
//    // 銆愭柊澧?1銆? 蹇呴』鏄惧紡娓呴櫎閿欒鏍囧織浣嶏紝鍚﹀垯涓插彛鍙兘浼氶攣姝?
//    // 鐗瑰埆鏄?ORE (婧㈠嚭閿欒)锛岃繖鏄皟璇曟椂鏈€瀹规槗閬囧埌鐨?
//    // ============================================================
//    uint32_t isr_flags = READ_REG(huart->Instance->SR); // 璇诲彇鐘舵€佸瘎瀛樺櫒(F4绯诲垪鏄疭R, H7/G4鏄疘SR)
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) || (isr_flags & UART_FLAG_ORE))
//    {
//        __HAL_UART_CLEAR_OREFLAG(huart); // 娓呴櫎婧㈠嚭閿欒
//    }
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_NE) || (isr_flags & UART_FLAG_NE))
//    {
//        __HAL_UART_CLEAR_NEFLAG(huart); // 娓呴櫎鍣０閿欒
//    }
//    
//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_FE) || (isr_flags & UART_FLAG_FE))
//    {
//        __HAL_UART_CLEAR_FEFLAG(huart); // 娓呴櫎甯ч敊璇?
//    }

//    if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_PE) || (isr_flags & UART_FLAG_PE))
//    {
//        __HAL_UART_CLEAR_PEFLAG(huart); // 娓呴櫎濂囧伓鏍￠獙閿欒
//    }

//    // ============================================================
//    // 銆愬師鏈夐€昏緫銆? 閬嶅巻瀹炰緥锛岄噸鍚?DMA 鎺ユ敹
//    // ============================================================
//    for (uint8_t i = 0; i < idx; ++i)
//    {
//        if (huart == usart_instance[i]->usart_handle)
//        {
//            // 灏濊瘯閲嶅惎鎺ユ敹 (浣跨敤 Idle Line 妫€娴嬫ā寮?
//            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, 
//                                         usart_instance[i]->recv_buff, 
//                                         usart_instance[i]->recv_buff_size);
//            
//            // 鍏抽棴 DMA 鍗婁紶杈撲腑鏂?(闃叉棰戠箒杩涘叆涓柇锛岄€氬父鎴戜滑鍙叧蹇冧紶瀹屾垨绌洪棽)
//            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
//            
//            // 鍙互鍔犱竴鍙?Log 鏂逛究璋冭瘯 (鍙€?
//            // LOGWARNING("[bsp_usart] Error recovered on instance idx [%d], ORE cleared.", i);
//            
//            return;
//        }
//    }
//}
