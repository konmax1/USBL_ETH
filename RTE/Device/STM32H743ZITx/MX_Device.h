/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 17/09/2018 11:14:57
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            64000000
#define MX_HSE_VALUE                            8000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLDSIFreq_Value                     500000000
#define MX_SYSCLKFreq_VALUE                     390000000
#define MX_HCLKFreq_Value                       195000000
#define MX_CortexFreq_Value                     390000000
#define MX_APB1Freq_Value                       97500000
#define MX_APB2Freq_Value                       97500000
#define MX_CECFreq_Value                        32000
#define MX_RTCFreq_Value                        1000000
#define MX_USBFreq_Value                        390000000
#define MX_WatchDogFreq_Value                   32000
#define MX_DSIFreq_Value                        96000000
#define MX_DSIPHYCLKFreq_Value                  96000000
#define MX_DSITXEscFreq_Value                   20000000
#define MX_SPDIFRXFreq_Value                    390000000
#define MX_MCO1PinFreq_Value                    64000000
#define MX_MCO2PinFreq_Value                    390000000

/*-------------------------------- CORTEX_M7  --------------------------------*/

#define MX_CORTEX_M7                            1

/* GPIO Configuration */

/*-------------------------------- DMA        --------------------------------*/

#define MX_DMA                                  1

/* NVIC Configuration */

/* NVIC DMA1_Stream4_IRQn */
#define MX_DMA1_Stream4_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream4_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream4_IRQn_Subriority         0

/* NVIC DMA1_Stream7_IRQn */
#define MX_DMA1_Stream7_IRQn_interruptPremptionPriority 2
#define MX_DMA1_Stream7_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream7_IRQn_Subriority         0

/* NVIC DMA1_Stream6_IRQn */
#define MX_DMA1_Stream6_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream6_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream6_IRQn_Subriority         0

/* NVIC DMA1_Stream2_IRQn */
#define MX_DMA1_Stream2_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream2_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream2_IRQn_Subriority         0

/* NVIC DMA2_Stream0_IRQn */
#define MX_DMA2_Stream0_IRQn_interruptPremptionPriority 1
#define MX_DMA2_Stream0_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA2_Stream0_IRQn_Subriority         0

/* NVIC DMA1_Stream5_IRQn */
#define MX_DMA1_Stream5_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream5_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream5_IRQn_Subriority         0

/* NVIC DMA1_Stream3_IRQn */
#define MX_DMA1_Stream3_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream3_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream3_IRQn_Subriority         0

/* NVIC DMA2_Stream1_IRQn */
#define MX_DMA2_Stream1_IRQn_interruptPremptionPriority 1
#define MX_DMA2_Stream1_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA2_Stream1_IRQn_Subriority         0

/* NVIC DMA1_Stream0_IRQn */
#define MX_DMA1_Stream0_IRQn_interruptPremptionPriority 0
#define MX_DMA1_Stream0_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_DMA1_Stream0_IRQn_Subriority         0

/*-------------------------------- ETH        --------------------------------*/

#define MX_ETH                                  1

/* GPIO Configuration */

/* Pin PA1 */
#define MX_ETH_REF_CLK_GPIO_Speed               GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_REF_CLK_Pin                      PA1
#define MX_ETH_REF_CLK_GPIOx                    GPIOA
#define MX_ETH_REF_CLK_GPIO_PuPd                GPIO_NOPULL
#define MX_ETH_REF_CLK_GPIO_Pin                 GPIO_PIN_1
#define MX_ETH_REF_CLK_GPIO_AF                  GPIO_AF11_ETH
#define MX_ETH_REF_CLK_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PA7 */
#define MX_ETH_CRS_DV_GPIO_Speed                GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_CRS_DV_Pin                       PA7
#define MX_ETH_CRS_DV_GPIOx                     GPIOA
#define MX_ETH_CRS_DV_GPIO_PuPd                 GPIO_NOPULL
#define MX_ETH_CRS_DV_GPIO_Pin                  GPIO_PIN_7
#define MX_ETH_CRS_DV_GPIO_AF                   GPIO_AF11_ETH
#define MX_ETH_CRS_DV_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PC4 */
#define MX_ETH_RXD0_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_RXD0_Pin                         PC4
#define MX_ETH_RXD0_GPIOx                       GPIOC
#define MX_ETH_RXD0_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_RXD0_GPIO_Pin                    GPIO_PIN_4
#define MX_ETH_RXD0_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_RXD0_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PC5 */
#define MX_ETH_RXD1_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_RXD1_Pin                         PC5
#define MX_ETH_RXD1_GPIOx                       GPIOC
#define MX_ETH_RXD1_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_RXD1_GPIO_Pin                    GPIO_PIN_5
#define MX_ETH_RXD1_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_RXD1_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PG11 */
#define MX_ETH_TX_EN_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TX_EN_Pin                        PG11
#define MX_ETH_TX_EN_GPIOx                      GPIOG
#define MX_ETH_TX_EN_GPIO_PuPd                  GPIO_NOPULL
#define MX_ETH_TX_EN_GPIO_Pin                   GPIO_PIN_11
#define MX_ETH_TX_EN_GPIO_AF                    GPIO_AF11_ETH
#define MX_ETH_TX_EN_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PA2 */
#define MX_ETH_MDIO_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_MDIO_Pin                         PA2
#define MX_ETH_MDIO_GPIOx                       GPIOA
#define MX_ETH_MDIO_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_MDIO_GPIO_Pin                    GPIO_PIN_2
#define MX_ETH_MDIO_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_MDIO_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PB13 */
#define MX_ETH_TXD1_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TXD1_Pin                         PB13
#define MX_ETH_TXD1_GPIOx                       GPIOB
#define MX_ETH_TXD1_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_TXD1_GPIO_Pin                    GPIO_PIN_13
#define MX_ETH_TXD1_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_TXD1_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PG13 */
#define MX_ETH_TXD0_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_ETH_TXD0_Pin                         PG13
#define MX_ETH_TXD0_GPIOx                       GPIOG
#define MX_ETH_TXD0_GPIO_PuPd                   GPIO_NOPULL
#define MX_ETH_TXD0_GPIO_Pin                    GPIO_PIN_13
#define MX_ETH_TXD0_GPIO_AF                     GPIO_AF11_ETH
#define MX_ETH_TXD0_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PC1 */
#define MX_ETH_MDC_GPIO_Speed                   GPIO_SPEED_FREQ_LOW
#define MX_ETH_MDC_Pin                          PC1
#define MX_ETH_MDC_GPIOx                        GPIOC
#define MX_ETH_MDC_GPIO_PuPd                    GPIO_NOPULL
#define MX_ETH_MDC_GPIO_Pin                     GPIO_PIN_1
#define MX_ETH_MDC_GPIO_AF                      GPIO_AF11_ETH
#define MX_ETH_MDC_GPIO_Mode                    GPIO_MODE_AF_PP

/* NVIC Configuration */

/* NVIC ETH_IRQn */
#define MX_ETH_IRQn_interruptPremptionPriority  3
#define MX_ETH_IRQn_PriorityGroup               NVIC_PRIORITYGROUP_4
#define MX_ETH_IRQn_Subriority                  0

/* NVIC ETH_WKUP_IRQn */
#define MX_ETH_WKUP_IRQn_interruptPremptionPriority 3
#define MX_ETH_WKUP_IRQn_PriorityGroup          NVIC_PRIORITYGROUP_4
#define MX_ETH_WKUP_IRQn_Subriority             0

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB8 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_Pin                         PB8
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_8
#define MX_I2C1_SCL_GPIO_FM8                    __NULL
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB9 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_Pin                         PB9
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_9
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD
#define MX_I2C1_SDA_GPIO_FM9                    __NULL

/* DMA Configuration */

/* DMA I2C1_TX */
#define MX_I2C1_TX_DMA_Instance                 DMA2_Stream0
#define MX_I2C1_TX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C1_TX_DMA_Priority                 DMA_PRIORITY_MEDIUM
#define MX_I2C1_TX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C1_TX_DMA_Mode                     DMA_NORMAL
#define MX_I2C1_TX_DMA_SyncRequestNumber        1
#define MX_I2C1_TX_DMA_Request                  DMA_REQUEST_I2C1_TX
#define MX_I2C1_TX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_I2C1_TX_DMA_Direction                DMA_MEMORY_TO_PERIPH
#define MX_I2C1_TX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_I2C1_TX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C1_TX_DMA_IpInstance               
#define MX_I2C1_TX_DMA_RequestNumber            1
#define MX_I2C1_TX_DMA_EventEnable              DISABLE
#define MX_I2C1_TX_DMA_SyncEnable               DISABLE
#define MX_I2C1_TX_DMA_DMA_Handle               
#define MX_I2C1_TX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C1_TX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_I2C1_TX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_I2C1_TX_DMA_PeriphInc                DMA_PINC_DISABLE

/* DMA I2C1_RX */
#define MX_I2C1_RX_DMA_Instance                 DMA2_Stream1
#define MX_I2C1_RX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C1_RX_DMA_Priority                 DMA_PRIORITY_MEDIUM
#define MX_I2C1_RX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C1_RX_DMA_Mode                     DMA_NORMAL
#define MX_I2C1_RX_DMA_SyncRequestNumber        1
#define MX_I2C1_RX_DMA_Request                  DMA_REQUEST_I2C1_RX
#define MX_I2C1_RX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_I2C1_RX_DMA_Direction                DMA_PERIPH_TO_MEMORY
#define MX_I2C1_RX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_I2C1_RX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C1_RX_DMA_IpInstance               
#define MX_I2C1_RX_DMA_RequestNumber            1
#define MX_I2C1_RX_DMA_EventEnable              DISABLE
#define MX_I2C1_RX_DMA_SyncEnable               DISABLE
#define MX_I2C1_RX_DMA_DMA_Handle               
#define MX_I2C1_RX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C1_RX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_I2C1_RX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_I2C1_RX_DMA_PeriphInc                DMA_PINC_DISABLE

/* NVIC Configuration */

/* NVIC I2C1_EV_IRQn */
#define MX_I2C1_EV_IRQn_interruptPremptionPriority 2
#define MX_I2C1_EV_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_EV_IRQn_Subriority              0

/* NVIC I2C1_ER_IRQn */
#define MX_I2C1_ER_IRQn_interruptPremptionPriority 2
#define MX_I2C1_ER_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C1_ER_IRQn_Subriority              0

/*-------------------------------- I2C2       --------------------------------*/

#define MX_I2C2                                 1

/* GPIO Configuration */

/* Pin PF0 */
#define MX_I2C2_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C2_SDA_Pin                         PF0
#define MX_I2C2_SDA_GPIOx                       GPIOF
#define MX_I2C2_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C2_SDA_GPIO_Pin                    GPIO_PIN_0
#define MX_I2C2_SDA_GPIO_AF                     GPIO_AF4_I2C2
#define MX_I2C2_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PF1 */
#define MX_I2C2_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_I2C2_SCL_Pin                         PF1
#define MX_I2C2_SCL_GPIOx                       GPIOF
#define MX_I2C2_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C2_SCL_GPIO_Pin                    GPIO_PIN_1
#define MX_I2C2_SCL_GPIO_AF                     GPIO_AF4_I2C2
#define MX_I2C2_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* DMA Configuration */

/* DMA I2C2_TX */
#define MX_I2C2_TX_DMA_Instance                 DMA1_Stream7
#define MX_I2C2_TX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C2_TX_DMA_Priority                 DMA_PRIORITY_MEDIUM
#define MX_I2C2_TX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C2_TX_DMA_Mode                     DMA_NORMAL
#define MX_I2C2_TX_DMA_SyncRequestNumber        1
#define MX_I2C2_TX_DMA_Request                  DMA_REQUEST_I2C2_TX
#define MX_I2C2_TX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_I2C2_TX_DMA_Direction                DMA_MEMORY_TO_PERIPH
#define MX_I2C2_TX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_I2C2_TX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C2_TX_DMA_IpInstance               
#define MX_I2C2_TX_DMA_RequestNumber            1
#define MX_I2C2_TX_DMA_EventEnable              DISABLE
#define MX_I2C2_TX_DMA_SyncEnable               DISABLE
#define MX_I2C2_TX_DMA_DMA_Handle               
#define MX_I2C2_TX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C2_TX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_I2C2_TX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_I2C2_TX_DMA_PeriphInc                DMA_PINC_DISABLE

/* DMA I2C2_RX */
#define MX_I2C2_RX_DMA_Instance                 DMA1_Stream6
#define MX_I2C2_RX_DMA_FIFOMode                 DMA_FIFOMODE_DISABLE
#define MX_I2C2_RX_DMA_Priority                 DMA_PRIORITY_MEDIUM
#define MX_I2C2_RX_DMA_MemDataAlignment         DMA_MDATAALIGN_BYTE
#define MX_I2C2_RX_DMA_Mode                     DMA_NORMAL
#define MX_I2C2_RX_DMA_SyncRequestNumber        1
#define MX_I2C2_RX_DMA_Request                  DMA_REQUEST_I2C2_RX
#define MX_I2C2_RX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_I2C2_RX_DMA_Direction                DMA_PERIPH_TO_MEMORY
#define MX_I2C2_RX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_I2C2_RX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_I2C2_RX_DMA_IpInstance               
#define MX_I2C2_RX_DMA_RequestNumber            1
#define MX_I2C2_RX_DMA_EventEnable              DISABLE
#define MX_I2C2_RX_DMA_SyncEnable               DISABLE
#define MX_I2C2_RX_DMA_DMA_Handle               
#define MX_I2C2_RX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_BYTE
#define MX_I2C2_RX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_I2C2_RX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_I2C2_RX_DMA_PeriphInc                DMA_PINC_DISABLE

/* NVIC Configuration */

/* NVIC I2C2_ER_IRQn */
#define MX_I2C2_ER_IRQn_interruptPremptionPriority 2
#define MX_I2C2_ER_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C2_ER_IRQn_Subriority              0

/* NVIC I2C2_EV_IRQn */
#define MX_I2C2_EV_IRQn_interruptPremptionPriority 2
#define MX_I2C2_EV_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_I2C2_EV_IRQn_Subriority              0

/*-------------------------------- RTC        --------------------------------*/

#define MX_RTC                                  1

/* GPIO Configuration */

/*-------------------------------- SPI1       --------------------------------*/

#define MX_SPI1                                 1

/* GPIO Configuration */

/* Pin PA4 */
#define MX_SPI1_NSS_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_SPI1_NSS_Pin                         PA4
#define MX_SPI1_NSS_GPIOx                       GPIOA
#define MX_SPI1_NSS_GPIO_PuPd                   GPIO_NOPULL
#define MX_SPI1_NSS_GPIO_Pin                    GPIO_PIN_4
#define MX_SPI1_NSS_GPIO_AF                     GPIO_AF5_SPI1
#define MX_SPI1_NSS_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PA6 */
#define MX_SPI1_MISO_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_SPI1_MISO_Pin                        PA6
#define MX_SPI1_MISO_GPIOx                      GPIOA
#define MX_SPI1_MISO_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI1_MISO_GPIO_Pin                   GPIO_PIN_6
#define MX_SPI1_MISO_GPIO_AF                    GPIO_AF5_SPI1
#define MX_SPI1_MISO_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PB5 */
#define MX_SPI1_MOSI_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_SPI1_MOSI_Pin                        PB5
#define MX_SPI1_MOSI_GPIOx                      GPIOB
#define MX_SPI1_MOSI_GPIO_PuPd                  GPIO_NOPULL
#define MX_SPI1_MOSI_GPIO_Pin                   GPIO_PIN_5
#define MX_SPI1_MOSI_GPIO_AF                    GPIO_AF5_SPI1
#define MX_SPI1_MOSI_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PA5 */
#define MX_SPI1_SCK_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_SPI1_SCK_Pin                         PA5
#define MX_SPI1_SCK_GPIOx                       GPIOA
#define MX_SPI1_SCK_GPIO_PuPd                   GPIO_NOPULL
#define MX_SPI1_SCK_GPIO_Pin                    GPIO_PIN_5
#define MX_SPI1_SCK_GPIO_AF                     GPIO_AF5_SPI1
#define MX_SPI1_SCK_GPIO_Mode                   GPIO_MODE_AF_PP

/* DMA Configuration */

/* DMA SPI1_RX */
#define MX_SPI1_RX_DMA_MemBurst                 DMA_MBURST_SINGLE
#define MX_SPI1_RX_DMA_Instance                 DMA1_Stream2
#define MX_SPI1_RX_DMA_FIFOMode                 DMA_FIFOMODE_ENABLE
#define MX_SPI1_RX_DMA_Priority                 DMA_PRIORITY_HIGH
#define MX_SPI1_RX_DMA_MemDataAlignment         DMA_MDATAALIGN_HALFWORD
#define MX_SPI1_RX_DMA_Mode                     DMA_NORMAL
#define MX_SPI1_RX_DMA_SyncRequestNumber        1
#define MX_SPI1_RX_DMA_Request                  DMA_REQUEST_SPI1_RX
#define MX_SPI1_RX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_SPI1_RX_DMA_Direction                DMA_PERIPH_TO_MEMORY
#define MX_SPI1_RX_DMA_PeriphBurst              DMA_PBURST_SINGLE
#define MX_SPI1_RX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_SPI1_RX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_SPI1_RX_DMA_IpInstance               
#define MX_SPI1_RX_DMA_RequestNumber            1
#define MX_SPI1_RX_DMA_EventEnable              DISABLE
#define MX_SPI1_RX_DMA_SyncEnable               DISABLE
#define MX_SPI1_RX_DMA_DMA_Handle               
#define MX_SPI1_RX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_HALFWORD
#define MX_SPI1_RX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_SPI1_RX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_SPI1_RX_DMA_PeriphInc                DMA_PINC_DISABLE
#define MX_SPI1_RX_DMA_FIFOThreshold            DMA_FIFO_THRESHOLD_FULL

/* DMA SPI1_TX */
#define MX_SPI1_TX_DMA_MemBurst                 DMA_MBURST_SINGLE
#define MX_SPI1_TX_DMA_Instance                 DMA1_Stream3
#define MX_SPI1_TX_DMA_FIFOMode                 DMA_FIFOMODE_ENABLE
#define MX_SPI1_TX_DMA_Priority                 DMA_PRIORITY_HIGH
#define MX_SPI1_TX_DMA_MemDataAlignment         DMA_MDATAALIGN_HALFWORD
#define MX_SPI1_TX_DMA_Mode                     DMA_NORMAL
#define MX_SPI1_TX_DMA_SyncRequestNumber        1
#define MX_SPI1_TX_DMA_Request                  DMA_REQUEST_SPI1_TX
#define MX_SPI1_TX_DMA_SyncPolarity             HAL_DMAMUX_SYNC_NO_EVENT
#define MX_SPI1_TX_DMA_Direction                DMA_MEMORY_TO_PERIPH
#define MX_SPI1_TX_DMA_PeriphBurst              DMA_PBURST_SINGLE
#define MX_SPI1_TX_DMA_SignalID                 HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_SPI1_TX_DMA_MemInc                   DMA_MINC_ENABLE
#define MX_SPI1_TX_DMA_IpInstance               
#define MX_SPI1_TX_DMA_RequestNumber            1
#define MX_SPI1_TX_DMA_EventEnable              DISABLE
#define MX_SPI1_TX_DMA_SyncEnable               DISABLE
#define MX_SPI1_TX_DMA_DMA_Handle               
#define MX_SPI1_TX_DMA_PeriphDataAlignment      DMA_PDATAALIGN_HALFWORD
#define MX_SPI1_TX_DMA_Polarity                 HAL_DMAMUX_REQ_GEN_RISING
#define MX_SPI1_TX_DMA_SyncSignalID             HAL_DMAMUX1_SYNC_EXTI0
#define MX_SPI1_TX_DMA_PeriphInc                DMA_PINC_DISABLE
#define MX_SPI1_TX_DMA_FIFOThreshold            DMA_FIFO_THRESHOLD_FULL

/* NVIC Configuration */

/* NVIC SPI1_IRQn */
#define MX_SPI1_IRQn_interruptPremptionPriority 2
#define MX_SPI1_IRQn_PriorityGroup              NVIC_PRIORITYGROUP_4
#define MX_SPI1_IRQn_Subriority                 0

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/* Pin PA13 */
#define MX_SYS_JTMS-SWDIO_Pin                   PA13

/* Pin PA14 */
#define MX_SYS_JTCK-SWCLK_Pin                   PA14

/* Pin PB3 */
#define MX_SYS_JTDO-SWO_Pin                     PB3

/*-------------------------------- TIM1       --------------------------------*/

#define MX_TIM1                                 1

/* GPIO Configuration */

/* Pin PA10 */
#define MX_S_TIM1_CH3_GPIO_ModeDefaultPP        GPIO_MODE_AF_PP
#define MX_S_TIM1_CH3_GPIO_Speed                GPIO_SPEED_FREQ_VERY_HIGH
#define MX_S_TIM1_CH3_Pin                       PA10
#define MX_S_TIM1_CH3_GPIOx                     GPIOA
#define MX_S_TIM1_CH3_GPIO_PuPd                 GPIO_NOPULL
#define MX_S_TIM1_CH3_GPIO_Pin                  GPIO_PIN_10
#define MX_S_TIM1_CH3_GPIO_AF                   GPIO_AF1_TIM1

/* Pin PE12 */
#define MX_TIM1_CH3N_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_TIM1_CH3N_GPIO_Speed                 GPIO_SPEED_FREQ_VERY_HIGH
#define MX_TIM1_CH3N_Pin                        PE12
#define MX_TIM1_CH3N_GPIOx                      GPIOE
#define MX_TIM1_CH3N_GPIO_PuPd                  GPIO_NOPULL
#define MX_TIM1_CH3N_GPIO_Pin                   GPIO_PIN_12
#define MX_TIM1_CH3N_GPIO_AF                    GPIO_AF1_TIM1

/* DMA Configuration */

/* DMA TIM1_CH3 */
#define MX_TIM1_CH3_DMA_Instance                DMA1_Stream0
#define MX_TIM1_CH3_DMA_FIFOMode                DMA_FIFOMODE_DISABLE
#define MX_TIM1_CH3_DMA_Priority                DMA_PRIORITY_VERY_HIGH
#define MX_TIM1_CH3_DMA_MemDataAlignment        DMA_MDATAALIGN_HALFWORD
#define MX_TIM1_CH3_DMA_Mode                    DMA_NORMAL
#define MX_TIM1_CH3_DMA_SyncRequestNumber       1
#define MX_TIM1_CH3_DMA_Request                 DMA_REQUEST_TIM1_CH3
#define MX_TIM1_CH3_DMA_SyncPolarity            HAL_DMAMUX_SYNC_NO_EVENT
#define MX_TIM1_CH3_DMA_Direction               DMA_MEMORY_TO_PERIPH
#define MX_TIM1_CH3_DMA_SignalID                HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_TIM1_CH3_DMA_MemInc                  DMA_MINC_ENABLE
#define MX_TIM1_CH3_DMA_IpInstance              
#define MX_TIM1_CH3_DMA_RequestNumber           1
#define MX_TIM1_CH3_DMA_EventEnable             DISABLE
#define MX_TIM1_CH3_DMA_SyncEnable              DISABLE
#define MX_TIM1_CH3_DMA_DMA_Handle              
#define MX_TIM1_CH3_DMA_PeriphDataAlignment     DMA_PDATAALIGN_HALFWORD
#define MX_TIM1_CH3_DMA_Polarity                HAL_DMAMUX_REQ_GEN_RISING
#define MX_TIM1_CH3_DMA_SyncSignalID            HAL_DMAMUX1_SYNC_EXTI0
#define MX_TIM1_CH3_DMA_PeriphInc               DMA_PINC_DISABLE

/* NVIC Configuration */

/* NVIC TIM1_TRG_COM_IRQn */
#define MX_TIM1_TRG_COM_IRQn_interruptPremptionPriority 2
#define MX_TIM1_TRG_COM_IRQn_PriorityGroup      NVIC_PRIORITYGROUP_4
#define MX_TIM1_TRG_COM_IRQn_Subriority         0

/* NVIC TIM1_UP_IRQn */
#define MX_TIM1_UP_IRQn_interruptPremptionPriority 2
#define MX_TIM1_UP_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_TIM1_UP_IRQn_Subriority              0

/* NVIC TIM1_BRK_IRQn */
#define MX_TIM1_BRK_IRQn_interruptPremptionPriority 2
#define MX_TIM1_BRK_IRQn_PriorityGroup          NVIC_PRIORITYGROUP_4
#define MX_TIM1_BRK_IRQn_Subriority             0

/* NVIC TIM1_CC_IRQn */
#define MX_TIM1_CC_IRQn_interruptPremptionPriority 2
#define MX_TIM1_CC_IRQn_PriorityGroup           NVIC_PRIORITYGROUP_4
#define MX_TIM1_CC_IRQn_Subriority              0

/*-------------------------------- UART4      --------------------------------*/

#define MX_UART4                                1

#define MX_UART4_VM                             VM_ASYNC

/* GPIO Configuration */

/* Pin PA11 */
#define MX_UART4_RX_GPIO_ModeDefaultPP          GPIO_MODE_AF_PP
#define MX_UART4_RX_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_UART4_RX_Pin                         PA11
#define MX_UART4_RX_GPIOx                       GPIOA
#define MX_UART4_RX_GPIO_PuPd                   GPIO_NOPULL
#define MX_UART4_RX_GPIO_Pin                    GPIO_PIN_11
#define MX_UART4_RX_GPIO_AF                     GPIO_AF6_UART4

/* Pin PA12 */
#define MX_UART4_TX_GPIO_ModeDefaultPP          GPIO_MODE_AF_PP
#define MX_UART4_TX_GPIO_Speed                  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_UART4_TX_Pin                         PA12
#define MX_UART4_TX_GPIOx                       GPIOA
#define MX_UART4_TX_GPIO_PuPd                   GPIO_NOPULL
#define MX_UART4_TX_GPIO_Pin                    GPIO_PIN_12
#define MX_UART4_TX_GPIO_AF                     GPIO_AF6_UART4

/* DMA Configuration */

/* DMA UART4_RX */
#define MX_UART4_RX_DMA_Instance                DMA1_Stream4
#define MX_UART4_RX_DMA_FIFOMode                DMA_FIFOMODE_DISABLE
#define MX_UART4_RX_DMA_Priority                DMA_PRIORITY_MEDIUM
#define MX_UART4_RX_DMA_MemDataAlignment        DMA_MDATAALIGN_BYTE
#define MX_UART4_RX_DMA_Mode                    DMA_NORMAL
#define MX_UART4_RX_DMA_SyncRequestNumber       1
#define MX_UART4_RX_DMA_Request                 DMA_REQUEST_UART4_RX
#define MX_UART4_RX_DMA_SyncPolarity            HAL_DMAMUX_SYNC_NO_EVENT
#define MX_UART4_RX_DMA_Direction               DMA_PERIPH_TO_MEMORY
#define MX_UART4_RX_DMA_SignalID                HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_UART4_RX_DMA_MemInc                  DMA_MINC_ENABLE
#define MX_UART4_RX_DMA_IpInstance              
#define MX_UART4_RX_DMA_RequestNumber           1
#define MX_UART4_RX_DMA_EventEnable             DISABLE
#define MX_UART4_RX_DMA_SyncEnable              DISABLE
#define MX_UART4_RX_DMA_DMA_Handle              
#define MX_UART4_RX_DMA_PeriphDataAlignment     DMA_PDATAALIGN_BYTE
#define MX_UART4_RX_DMA_Polarity                HAL_DMAMUX_REQ_GEN_RISING
#define MX_UART4_RX_DMA_SyncSignalID            HAL_DMAMUX1_SYNC_EXTI0
#define MX_UART4_RX_DMA_PeriphInc               DMA_PINC_DISABLE

/* DMA UART4_TX */
#define MX_UART4_TX_DMA_Instance                DMA1_Stream5
#define MX_UART4_TX_DMA_FIFOMode                DMA_FIFOMODE_DISABLE
#define MX_UART4_TX_DMA_Priority                DMA_PRIORITY_MEDIUM
#define MX_UART4_TX_DMA_MemDataAlignment        DMA_MDATAALIGN_BYTE
#define MX_UART4_TX_DMA_Mode                    DMA_NORMAL
#define MX_UART4_TX_DMA_SyncRequestNumber       1
#define MX_UART4_TX_DMA_Request                 DMA_REQUEST_UART4_TX
#define MX_UART4_TX_DMA_SyncPolarity            HAL_DMAMUX_SYNC_NO_EVENT
#define MX_UART4_TX_DMA_Direction               DMA_MEMORY_TO_PERIPH
#define MX_UART4_TX_DMA_SignalID                HAL_DMAMUX1_REQ_GEN_EXTI0
#define MX_UART4_TX_DMA_MemInc                  DMA_MINC_ENABLE
#define MX_UART4_TX_DMA_IpInstance              
#define MX_UART4_TX_DMA_RequestNumber           1
#define MX_UART4_TX_DMA_EventEnable             DISABLE
#define MX_UART4_TX_DMA_SyncEnable              DISABLE
#define MX_UART4_TX_DMA_DMA_Handle              
#define MX_UART4_TX_DMA_PeriphDataAlignment     DMA_PDATAALIGN_BYTE
#define MX_UART4_TX_DMA_Polarity                HAL_DMAMUX_REQ_GEN_RISING
#define MX_UART4_TX_DMA_SyncSignalID            HAL_DMAMUX1_SYNC_EXTI0
#define MX_UART4_TX_DMA_PeriphInc               DMA_PINC_DISABLE

/* NVIC Configuration */

/* NVIC UART4_IRQn */
#define MX_UART4_IRQn_interruptPremptionPriority 2
#define MX_UART4_IRQn_PriorityGroup             NVIC_PRIORITYGROUP_4
#define MX_UART4_IRQn_Subriority                0

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PC13 */
#define MX_PC13_Pin                             PC13
#define MX_PC13_GPIOx                           GPIOC
#define MX_PC13_GPIO_PuPd                       GPIO_NOPULL
#define MX_User_Btn                             PC13
#define MX_PC13_GPIO_Pin                        GPIO_PIN_13
#define MX_PC13_GPIO_ModeDefaultEXTI            GPIO_MODE_IT_RISING

/* Pin PB14 */
#define MX_PB14_GPIO_Speed                      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_PB14_Pin                             PB14
#define MX_PB14_GPIOx                           GPIOB
#define MX_PB14_PinState                        GPIO_PIN_RESET
#define MX_PB14_GPIO_PuPd                       GPIO_NOPULL
#define MX_LD3_RED                              PB14
#define MX_PB14_GPIO_Pin                        GPIO_PIN_14
#define MX_PB14_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE13 */
#define MX_PE13_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PE13_Pin                             PE13
#define MX_PE13_GPIOx                           GPIOE
#define MX_PE13_PinState                        GPIO_PIN_RESET
#define MX_PE13_GPIO_PuPd                       GPIO_NOPULL
#define MX_PE13_GPIO_Pin                        GPIO_PIN_13
#define MX_PE13_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PA0-WKUP */
#define MX_PA0_WKUP_Pin                         PA0_WKUP
#define MX_PA0_WKUP_GPIOx                       GPIOA
#define MX_PA0_WKUP_GPIO_PuPd                   GPIO_PULLDOWN
#define MX_PA0_WKUP_GPIO_Pin                    GPIO_PIN_0
#define MX_PA0_WKUP_GPIO_ModeDefaultEXTI        GPIO_MODE_IT_RISING_FALLING

/* Pin PB7 */
#define MX_PB7_GPIO_Speed                       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_PB7_Pin                              PB7
#define MX_PB7_GPIOx                            GPIOB
#define MX_PB7_PinState                         GPIO_PIN_RESET
#define MX_PB7_GPIO_PuPd                        GPIO_NOPULL
#define MX_LD2_Blue                             PB7
#define MX_PB7_GPIO_Pin                         GPIO_PIN_7
#define MX_PB7_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP
#define MX_PB7_GPIO_FM7                         __NULL

/* Pin PF2 */
#define MX_PF2_Pin                              PF2
#define MX_PF2_GPIOx                            GPIOF
#define MX_PF2_GPIO_PuPd                        GPIO_NOPULL
#define MX_Invensense_IRQ                       PF2
#define MX_PF2_GPIO_Pin                         GPIO_PIN_2
#define MX_PF2_GPIO_ModeDefaultEXTI             GPIO_MODE_IT_RISING

/* Pin PD0 */
#define MX_PD0_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PD0_Pin                              PD0
#define MX_PD0_GPIOx                            GPIOD
#define MX_PD0_PinState                         GPIO_PIN_RESET
#define MX_PD0_GPIO_PuPd                        GPIO_NOPULL
#define MX_V_invensense                         PD0
#define MX_PD0_GPIO_Pin                         GPIO_PIN_0
#define MX_PD0_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

#endif  /* __MX_DEVICE_H */

