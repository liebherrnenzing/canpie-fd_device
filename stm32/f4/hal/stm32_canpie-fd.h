/**
 * @file stm32_canpie-fd.h 
 * @brief header to stm32_canpie-fd.h module
 * @addtogroup 
 * @{
 */

#ifndef STM32_CANPIE_FD_H_
#define STM32_CANPIE_FD_H_

#ifdef __cplusplus
extern "C"
{
#endif // #ifdef __cplusplus

/*--------------------------------------------------------------------------*/
/* included files                                                           */
/*--------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

// external functions
// variables of module
extern CAN_HandleTypeDef hcan1;

/*--------------------------------------------------------------------------*/
/* general definitions                                                      */
/*--------------------------------------------------------------------------*/
#define HCAN1 hcan1

#ifdef __cplusplus
}// closing brace for extern "C"
#endif // #ifdef __cplusplus

#endif // #ifndef STM32_CANPIE_FD_H_

/** @} */
