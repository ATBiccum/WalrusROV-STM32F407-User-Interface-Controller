/* Walrus ROV
 * STM32F4 Control Station
 * 29/09/2022
 * Alexander, Tony, Clinton
 *
 * 
 * TODO:
 *  1. Get consistent wireless coms
 *  2. Test packet parsing and verification
 *  3. Apply parsed packets to motor control maps
 * 
 *
 * NEEDs:
 *  1. When no packet received: force motors to off
 *  2. 
 *
 * CONFIGS:
 *
 * GND = GND = 1
 * VCC = 3.3V = 2
 * MOSI = PC_12 = 6 
 * MISO = PC_11 = 7 
 * SCK = PC_10 = 5 
 * CSN = PC_9 = 4
 * CE = PC_8 = 3
 * IRQ = PC_7 = 8
 *
 * PS3 Controls Packet Format: 
 *  1  4   7  10   13  16  19 20 21 22 23 24 25 
 * "# 000 000 000 000 000 000 0  0  0  0  0  0" = 25
 * Pound, L1, L2, LeftHatX, LefthatY, RightHatX, RightHatY, R1, R2, Triangle, Circle, Cross, Square
 * 
 *
 * RESORUCES:
 * https://os.mbed.com/users/Owen/code/nRF24L01P/
 * https://os.mbed.com/cookbook/nRF24L01-wireless-transceiver
 * https://forums.mbed.com/t/hitchhikers-guide-to-printf-in-mbed-6/12492
 * https://controllerstech.com/how-to-setup-uart-using-registers-in-stm32/
 * https://arduino.ua/docs/AfroESC30A.pdf
 * https://os.mbed.com/handbook/PwmOut
 * https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 * https://os.mbed.com/docs/mbed-os/v6.15/apis/thread.html
 *
 *
 *
 * enc28j60
 */

#include <mbed.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "nRF24L01P.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

UART_HandleTypeDef huart2; //Serial Monitor
SPI_HandleTypeDef hspi3; //Tranceivers

Thread nRF24Thread;

uint8_t uartDataBuf[32] = {0};
uint8_t uartData[32] = {0};
size_t old_posRx;
size_t posRx;
bool newData = false;

//nRF Tx and Rx Variables
char nRF_RxData[32] = {0}; //"#99000000134111154132000000999#"
int nRF_RxDataCnt = 0;
char nRF_TxData[32] = {0};
char nRF_TxDataBuf[32] = {0};
bool nRF_NewData = false;
bool nRF_Initialized = false;

int main()
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_Init();             
    SystemClock_Config();   
    GPIO_Init();
    USART2_UART_Init();
    USART3_UART_Init();
    SPI3_Init();
        
    nRF24Thread.start(nRF24);

    while(1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        HAL_Delay(250);
        printf("nRF Transmit Data: %s | nRF Receive Data: %s \n", nRF_TxData, nRF_RxData);
    }
}

//Check sum stuff: 
//Function 1: Checksum calc = Parse end 3 digits - 1 -> Add rest of data together -> return added value (checksum)
//Function 2: Compare checksum = Parse end 3 digits - 1 -> call checksumcalc and save value -> if == memcpy to nRF_RxData + counter at start
//if not dont memcpy print bad packet or something

/*char checkSum(char data[32])
{
    int checkSumCalc;

    for(int i = 1; i < 29; i++)
    {
        checkSumCalc += data[i];
    }

    memcpy(data[29], checkSumCalc, 3);
    return data;
}
*/

void USART_Process_Data() 
{
    memcpy(nRF_TxData, uartDataBuf, 32);
}

void nRF24()
{
    nRF24L01P nRF(PC_12, PC_11, PC_10, PC_9, PC_8, PC_7); //mosi, miso, sck, csn, ce, irq
    
    while(1)
    {
        /*****Transceiver Initialization*****/
        /* Re-try nRF Initialization Steps 
        1. Enable power to module  
        2. Set transfer size
        3. Check output from nRF config functions
            If No Error
            Module is correctly initialized, proceed to step 4
            If Error:
            Disable power to module, restart at step 1
        4. Set receive mode (module will change to tx mode when transmitting)
        5. Enable the module 
        */
        HAL_Delay(5000);
        nRF.powerUp();
        nRF.setTransferSize(32);
        uint8_t nRF_InitCheck = 0;
        //Check if nRF module is powered and communicating properly; if not restart this thread
        if(nRF.getRfFrequency() < 2525 || nRF.getRfFrequency() > 2400){printf("nRF24L01+ Frequency     : %d MHz\r\n",  nRF.getRfFrequency());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(1);}
        if(nRF.getRfOutputPower() != 1){printf("nRF24L01+ Output power  : %d dBm\r\n",  nRF.getRfOutputPower());nRF_InitCheck++;}
        else{nRF.getRfOutputPower();nRF.disable();nRF.powerDown();nRF_Error_Handler(2);}
        if(nRF.getAirDataRate() != 0){printf("nRF24L01+ Data Rate     : %d kbps\r\n", nRF.getAirDataRate());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(3);}
        if(nRF.getTxAddress() != 0){printf("nRF24L01+ TX Address    : 0x%010llX\r\n", nRF.getTxAddress());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(4);}
        if(nRF.getRxAddress() != 0){printf("nRF24L01+ RX Address    : 0x%010llX\r\n", nRF.getRxAddress());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(5);}
        if(nRF.getTransferSize() == 32){printf("nRF24L01+ Transfer Size : %d\r\n", nRF.getTransferSize());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(6);}
        if(nRF.getCrcWidth() != 0){printf("nRF24L01P+ CRC Width    :  %d\r\n", nRF.getCrcWidth());nRF_InitCheck++;}
        else{nRF.disable();nRF.powerDown();nRF_Error_Handler(7);}

        if (nRF_InitCheck == 7)
        {
            nRF.setReceiveMode(); 
            nRF.enable();
            printf("nRF Initialized!\n"); 
            nRF_Initialized = true;
        }
        while (nRF_Initialized)
        {
            thread_sleep_for(10); //Transmit every 100ms

            //Receive data if there is data to be received
            if (nRF.readable())
            {
                nRF_RxDataCnt = nRF.read(NRF24L01P_PIPE_P0, nRF_RxData, sizeof(nRF_RxData));
                nRF_NewData = true;
            }
            if (nRF_NewData == true)
            {
                //printf("nRF Received Data: %s\n", nRF_RxData);
                nRF_NewData = false;
            }
            memcpy(nRF_TxData, uartData, 32);
            nRF.write(NRF24L01P_PIPE_P1, nRF_TxData, sizeof(nRF_TxData));
            
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // Initializes the RCC Oscillators according to the specified parameters
    // in the RCC_OscInitTypeDef structure.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    Error_Handler();
    }
    // Initializes the CPU, AHB and APB buses clocks
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
    Error_Handler();
    }
}

static void SPI3_Init(void)
{
  //SPI3 parameter configuration
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void USART2_UART_Init(void)
{
    //Initialize USART2 for Serial Monitor (Using HAL, will switch to LL)
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_GPIO_InitTypeDef LL_GPIO_InitStruct = {0};

    //GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    //Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    //Configure GPIO pins:
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Configure GPIO pins for UART3
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    LL_GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    LL_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    LL_GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &LL_GPIO_InitStruct);
}

void Error_Handler(void)
{
    //User can add his own implementation to report the HAL error return state
    __disable_irq();
    while (1)
    {
    }
}

static void USART3_UART_Init(void)
{
    //Initialize USART3 for DMA Rx and Tx
    LL_USART_InitTypeDef USART_InitStruct = {0};
    
    //Peripheral clock enable
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    //USART3 DMA Init for Rx
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)uartData);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ARRAY_LEN(uartData));

    //DMA interrupt init for Rx
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    //USART Config (Tx done in DMA transmit function)
    USART_InitStruct.BaudRate = 9600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableIT_IDLE(USART3);

    //USART interrupt init 
    NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART3_IRQn);

    //Enable USART3 and DMA Rx Stream 
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART3);
}

void nRF_Error_Handler(uint8_t value)
{
    switch (value)
    {
        case 1:
        //RF Frequency Error
            printf("nRF24L01P: Unknown RF Frequency value.\n");
            break;
        case 2:
        //RF Output Power Error
            printf("nRF24L01P: Unknown RF Output Power value.\n");
            break;
        case 3:
        //Air Data Rate Error
            printf("nRF24L01P: Unknown Air Data Rate value.\n");
            break;
        case 4:
        //Transmit Address Error
            printf("nRF24L01P: Unknown Transmit Address value.\n");
            break;
        case 5:
        //Receive Address Error
            printf("nRF24L01P: Unknown Receive Address value.\n");
            break;
        case 6:
        //Transfer Size Error
            printf("nRF24L01P: Unknown Transfer Size value.\n");
            break;
        case 7:
        //CRC Size Error 
            printf("nRF24L01P: Unknown CRC Width value.\n");
            break;
    }
}

void USART3_IRQHandler(void) 
{
    //Check idle line interrupt for USART
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
        LL_USART_ClearFlag_IDLE(USART3);        //Clear IDLE line Flag
        USART_Process_Data();                       //Filter received data
    }
}

void DMA1_Stream1_IRQHandler(void) 
{
    // Check transfer-complete interrupt for Rx
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) 
    {
        LL_DMA_ClearFlag_TC1(DMA1);             //Clear transfer complete flag 
        USART_Process_Data();                       //Filter received data
    }
}