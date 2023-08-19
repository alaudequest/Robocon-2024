#include "main.h"
#ifdef CONFIG_USE_MCT8316
#ifndef _MCT8316_H
#define _MCT8316_H

/**
  * @brief MCT8216 address.
  */
#define MCT8316_ADDRESS                  0x00

/********************************************************************/
/**
 * @name  EEPROM Access and I2C Interface 
 * @brief MCT8316 has 1024 bits of EEPROM, which are used to store the motor configuration parameters. EEPROM
 *        can be written and read by I2C serial interface 
 * @note  MCT8316 allows EEPROM write and read operations only when the motor is not spinning.
 */
#define REG_ISD_CONFIG                  0x80     
#define REG_MOTOR_STARTUP1              0x82        
#define REG_MOTOR_STARTUP2              0x84        
#define REG_CLOSED_LOOP1                0x86
#define REG_CLOSED_LOOP2                0x88
#define REG_CLOSED_LOOP3                0x8A
#define REG_CLOSED_LOOP4                0x8C
#define REG_CONST_SPEED                 0x8E
#define REG_CONST_PWR                   0x90
#define REG_FAULT_CONFIG1               0x92
#define REG_FAULT_CONFIG2               0x94
#define REG_TRAP_CONFIG1                0x9A
#define REG_TRAP_CONFIG2                0x9C
#define REG_150_DEG_TWO_PH_PROFILE      0x96
#define REG_150_DEG_THREE_PH_PROFILE    0x98
#define REG_PIN_CONFIG1                 0xA4
#define REG_PIN_CONFIG2                 0xA6
#define REG_DEVICE_CONFIG               0xA8
#define REG_PERIPH_CONFIG               0xAA
#define REG_GD_CONFIG1                  0xAC
#define REG_GD_CONFIG2                  0xAE
#define REG_GATE_DRIVER_FAULT_STATUS    0xE0
#define REG_CONTROLLER_FAULT_STATUS     0xE2
#define REG_SYS_STATUS1                 0xE4
#define REG_SYS_STATUS2                 0xEA
#define REG_SYS_STATUS3                 0xEC
#define REG_ALGO_CTRL1                  0xE6
#define REG_DEVICE_CTRL                 0xE8

/** @addtogroup EEPROM_Register_Map
  * @{
  */

/**************** ALGORITHM_CONFIGUARTION register ****************/

#define REG_ISD_CONFIG                  0x80     
#define REG_MOTOR_STARTUP1              0x82        
#define REG_MOTOR_STARTUP2              0x84        
#define REG_CLOSED_LOOP1                0x86
#define REG_CLOSED_LOOP2                0x88
#define REG_CLOSED_LOOP3                0x8A
#define REG_CLOSED_LOOP4                0x8C
#define REG_CONST_SPEED                 0x8E
#define REG_CONST_PWR                   0x90
#define REG_150_DEG_TWO_PH_PROFILE      0x96
#define REG_150_DEG_THREE_PH_PROFILE    0x98
#define REG_TRAP_CONFIG1                0x9A
#define REG_TRAP_CONFIG2                0x9C

/**************** Bit definition for REG_ISD_CONFIG register ****************/
#define ISD_EN_Pos                          (31UL)
#define ISD_EN_Msk                          (0x1UL << ISD_EN_Pos)
#define ISD_EN                              ISD_EN_Msk
#define ISD_BRAKE_EN_Pos                    (30UL)
#define ISD_BRAKE_EN_Msk                    (0x1UL << ISD_BRAKE_EN_Pos)
#define ISD_BRAKE_EN                        ISD_BRAKE_EN_Msk
#define ISD_HIZ_EN_Pos                      (29UL)
#define ISD_HIZ_EN_Msk                      (0x1UL << ISD_HIZ_EN_Pos)
#define ISD_HIZ_EN                          ISD_HIZ_EN_Msk
#define ISD_RVS_EN_Pos                      (28UL)
#define ISD_RVS_EN_Msk                      (0x1UL << ISD_RVS_EN_Pos)
#define ISD_RVS_DR_EN                       ISD_RVS_EN_Msk
#define ISD_RESYNC_EN_Pos                   (27UL)
#define ISD_RESYNC_EN_Msk                   (0x1UL << ISD_RESYNC_EN_Pos)
#define ISD_RESYNC_EN                       ISD_RESYNC_EN_Msk
#define ISD_STAT_BRK_EN_Pos                 (26UL)
#define ISD_STAT_BRK_EN_Pos                 (0x1UL << ISD_STAT_BRK_EN_Pos)
#define ISD_STAT_BRK_EN                     ISD_STAT_BRK_EN_Pos

#define ISD_STAT_DETECT_THR_Pos             (22UL)
#define ISD_STAT_DETECT_THR_Msk             (0x7UL << ISD_STAT_DETECT_THR_Pos)
#define ISD_STAT_DETECT_THR_5mV             (0x0UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_10mV            (0x1UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_15mV            (0x2UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_20mV            (0x3UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_25mV            (0x4UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_30mV            (0x5UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_50mV            (0x6UL << ISD_STAT_DETECT_THR_Pos)        
#define ISD_STAT_DETECT_THR_1000mV          ISD_STAT_DETECT_THR_Msk                  

#define ISD_BRK_MODE_Pos                    (21UL)
#define ISD_BRK_MODE_Msk                    (0x1UL << ISD_BRK_MODE_Pos)
#define ISD_ISD_BRK_MODE_LOW                (0x0UL << ISD_BRK_MODE_Pos)             
#define ISD_ISD_BRK_MODE_HIGH               ISD_BRK_MODE_Msk                        

#define ISD_BRK_CONF_Pos                    (20UL)
#define ISD_BRK_CONF_Msk                    (0x1UL << ISD_BRK_CONF_Pos)
#define ISD_BRK_TIME                        (0x0UL << ISD_BRK_CONF_Pos)
#define ISD_BRK_CURR_THR                    ISD_BRK_CONF_Msk

#define ISD_BRK_CURR_THR_Pos                (17UL)
#define ISD_BRK_CURR_THR_Msk                (0x7UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_5mV                (0x0UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_10mV               (0x1UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_15mV               (0x2UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_20mV               (0x3UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_25mV               (0x4UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_30mV               (0x5UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_50mV               (0x6UL << ISD_BRK_CURR_THR_Pos)
#define ISD_BRK_CURR_THR_100mV              ISD_BRK_CURR_THR_Msk

#define ISD_BRK_TIME_Pos                    (13UL)
#define ISD_BRK_TIME_Msk                    (0xFUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_10mS                   (0x0UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_50mS                   (0x1UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_100mS                  (0x2UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_200mS                  (0x3UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_300mS                  (0x4UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_400mS                  (0x5UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_500mS                  (0x6UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_750mS                  (0x7UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_1S                     (0x8UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_2S                     (0x9UL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_3S                     (0xAUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_4S                     (0xBUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_5S                     (0xCUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_7S5                    (0xDUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_10S                    (0xEUL << ISD_BRK_TIME_Pos)
#define ISD_BRK_TIME_15S                    ISD_BRK_TIME_Msk

#define ISD_HIZ_TIME_Pos                    (9UL)
#define ISD_HIZ_TIME_Msk                    (0xFUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_10mS                   (0x0UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_50mS                   (0x1UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_100mS                  (0x2UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_200mS                  (0x3UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_300mS                  (0x4UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_400mS                  (0x5UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_500mS                  (0x6UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_750mS                  (0x7UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_1S                     (0x8UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_2S                     (0x9UL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_3S                     (0xAUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_4S                     (0xBUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_5S                     (0xCUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_7S5                    (0xDUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_10S                    (0xEUL << ISD_HIZ_TIME_Pos)
#define ISD_HIZ_TIME_15S                    ISD_HIZ_TIME_Msk

#define ISD_STARTUP_BRK_TIME_Pos            (6UL)
#define ISD_STARTUP_BRK_TIME_Msk            (0x7UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_1mS            (0x0UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_10mS           (0x1UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_25mS           (0x2UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_50mS           (0x3UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_100mS          (0x4UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_250mS          (0x5UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_500mS          (0x6UL << ISD_STARTUP_BRK_TIME_Pos)
#define ISD_STARTUP_BRK_TIME_1000mS         ISD_STARTUP_BRK_TIME_Msk

#define ISD_RESYNC_MIN_THRESHOLD_Pos        (3UL)
#define ISD_RESYNC_MIN_THRESHOLD_Msk        (0x7UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_MIN_DUTY   (0x0UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_300mV      (0x1UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_400mV      (0x2UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_500mV      (0x3UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_600mV      (0x4UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_800mV      (0x5UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_1V         (0x6UL << ISD_RESYNC_MIN_THRESHOLD_Pos)
#define ISD_RESYNC_MIN_THRESHOLD_1V25       ISD_RESYNC_MIN_THRESHOLD_Msk


#endif