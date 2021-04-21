/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_RX_LEN  200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 uint8_t bl_rx_buffer[BL_RX_LEN];

 uint8_t supported_commands[] = {
                                BL_GET_VER ,
                                BL_GET_HELP,
                                BL_GET_CID,
                                BL_GET_RDP_STATUS,
                                BL_GO_TO_ADDR,
                                BL_FLASH_ERASE,
                                BL_MEM_WRITE,
                                BL_READ_SECTOR_P_STATUS} ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
//  char *data = "harry from uart1\r\n";
//  char *data1 = "harry from uart2\r\n";
  RetargetInit(&huart3);
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_SET) {
		 printf("we are in a bootloader mode \r\n");
		 bootloader_uart_read_data();
  } else {
		 printf("we are in a application \r\n");
		 bootloader_jump_to_user_app();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Transmit(&huart1,(uint8_t*)data,strlen(data),HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)data1,strlen(data1),HAL_MAX_DELAY);
//	 printf("welcome to the app \r\n");

  }
  /* USER CODE END 3 */
}

void bootloader_uart_read_data(void) {
	uint8_t rcv_len = 0;
	while(1){
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
        HAL_UART_Receive(&huart1,bl_rx_buffer,1,HAL_MAX_DELAY);
        rcv_len= bl_rx_buffer[0];
        HAL_UART_Receive(&huart1,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
        switch(bl_rx_buffer[1]) {
        case BL_GET_VER:
            bootloader_handle_getver_cmd(bl_rx_buffer);
            break;
        case BL_GET_HELP:
            bootloader_handle_gethelp_cmd(bl_rx_buffer);
            break;
        case BL_GET_CID:
            bootloader_handle_getcid_cmd(bl_rx_buffer);
            break;
        case BL_GET_RDP_STATUS:
            bootloader_handle_getrdp_cmd(bl_rx_buffer);
            break;
        case BL_GO_TO_ADDR:
            bootloader_handle_go_cmd(bl_rx_buffer);
            break;
        case BL_FLASH_ERASE:
            bootloader_handle_flash_erase_cmd(bl_rx_buffer);
            break;
        case BL_MEM_WRITE:
            bootloader_handle_mem_write_cmd(bl_rx_buffer);
            break;
         default:
            printf("Invalid command code received from host \r\n");
            break;
        }
	}
}

/*code to jump to user application
 *Here we are assuming FLASH_SECTOR2_BASE_ADDRESS
 *is where the user application is stored
 */
void bootloader_jump_to_user_app(void)
{

   //just a function pointer to hold the address of the reset handler of the user app.
    void (*app_reset_handler)(void);

    // 1. configure the MSP by reading the value from the base address of the sector 2
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    printf("MSP value : %#x \r\n",msp_value);

    //This function comes from CMSIS.
    __set_MSP(msp_value);

//    SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

    /* 2. Now fetch the reset handler address of the user application
     * from the location FLASH_SECTOR2_BASE_ADDRESS+4
     */
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;

    printf("app reset handler addr : %#x \r\n",app_reset_handler);

    //3. jump to reset handler of the user application
    app_reset_handler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**************Implementation of Boot-loader Command Handle functions *********/


/*Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) verify the checksum
      printf("bootloader Getting version ... \r\n");

	 //Total length of the command packet
	  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	  //extract the CRC32 sent by the Host
	  uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
        printf("checksum success !! \r\n");
        // checksum is correct..
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version=get_bootloader_version();
        printf("Bootloader version  : %d %#x \r\n",bl_version,bl_version);
        bootloader_uart_write_data(&bl_version,1);

    }else
    {
        printf("checksum fail !!\n");
        //checksum is wrong send nack
        bootloader_send_nack();
    }


}


/*This function sends ACK if CRC matches along with "len to follow"*/

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len) {
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(&huart1,ack_buf,2,HAL_MAX_DELAY);

}

/*This function sends NACK */
void bootloader_send_nack(void) {
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart1,&nack,1,HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}
/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(&huart1,pBuffer,len,HAL_MAX_DELAY);

}

//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}


/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    printf("Geting Supported commands.... \r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printf("checksum success !! \r\n");
        bootloader_send_ack(pBuffer[0],sizeof(supported_commands));
        bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}else {
        printf("checksum fail !! \r\n");
        bootloader_send_nack();
	}

}


/*Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	printf("Getting Chip Id ...\r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printf("checksum success !! \r\n");
        bootloader_send_ack(pBuffer[0],2);
        bl_cid_num = get_mcu_chip_id();
        printf("MCU id : %d %#x !! \r\n",bl_cid_num, bl_cid_num);
        bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);

	}else
	{
        printf("checksum fail !!\n");
        bootloader_send_nack();
	}

}


//Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id(void)
{
/*
	The STM32L100C discovery board MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus port (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;

}


/*Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    printf("bootloader Getting RDP leval \r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printf("checksum success !! \r\n");
        bootloader_send_ack(pBuffer[0],1);
        rdp_level = get_flash_rdp_level();
        printf("RDP level: %d %#x \r\n",rdp_level,rdp_level);
        bootloader_uart_write_data(&rdp_level,1);

	}else
	{
        printf("checksum fail !! \r\n");
        bootloader_send_nack();
	}


}


/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 9. Description of the option bytes" in stm32f446xx RM
 */
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
#if 1
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	 rdp_status =  (uint8_t)(*pOB_addr >> 8) ;
#endif

	return rdp_status;

}

/*Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    printf("Going to address.. \r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printf("checksum success !! \r\n");

        bootloader_send_ack(pBuffer[0],1);

        //extract the go address
        go_address = *((uint32_t *)&pBuffer[2] );
        printf("GO addr: %#x \r\n",go_address);

        if( verify_address(go_address) == ADDR_VALID )
        {
            //tell host that address is fine
            bootloader_uart_write_data(&addr_valid,1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

//            go_address+=1; //make T bit =1

            void (*lets_jump)(void) = (void *)go_address;

            printf("jumping to go address! \r\n");

            lets_jump();

		}else
		{
            printf("GO addr invalid ! \r\n");
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}

	}else
	{
        printf("checksum fail !! \r\n");
        bootloader_send_nack();
	}


}

//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{

	if ( go_address >= SRAM_BASE && go_address <= SRAM_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}


/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    printf("Erasing flash.... \r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 8) ) ;
	printf("host crc is 0x%X \r\n",host_crc);

	if (!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-8,host_crc))
	{
        printf("checksum success !! \r\n");
        bootloader_send_ack(pBuffer[0],1);
        printf("initial_page : %d  no of page: %d \r\n",pBuffer[2],pBuffer[3]);
        uint32_t address_of_page= 0;
        memcpy(&address_of_page,&pBuffer[8],4);
        printf("address is 0x%X \r\n",address_of_page);
        HAL_GPIO_WritePin(MY_LED_GPIO_Port, MY_LED_Pin ,1);
        erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3] , address_of_page );
        HAL_GPIO_WritePin(MY_LED_GPIO_Port, MY_LED_Pin,0);

        printf("flash erase status: %#x \r\n",erase_status);

        bootloader_uart_write_data(&erase_status,1);

	}else
	{
        printf("checksum fail !! \r\n");
        bootloader_send_nack();
	}
}


uint8_t execute_flash_erase(uint8_t page_number , uint8_t number_of_page , uint32_t address) {
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t PageError;
	HAL_StatusTypeDef status;


	if( number_of_page > 1023U )
		return INVALID_PAGE;

	if( (page_number <= 1023) ) {
		    /*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_page = 1023 - page_number;
           if( number_of_page > remanining_page)
           {
           	number_of_page = remanining_page;
           }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_PAGES;
			flashErase_handle.NbPages = number_of_page; // this is the initial sector
		    flashErase_handle.PageAddress = address;
		    printf("Page erase address is 0x%X \r\n",flashErase_handle.PageAddress);

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &PageError);
		HAL_FLASH_Lock();

		return status;
	}
	return INVALID_PAGE;
}

/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];

    printf("bootloader_handle_mem_write_cmd \r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printf("checksum success !! \r\n");

        bootloader_send_ack(pBuffer[0],1);

        printf("mem write address : %#x \r\n",mem_address);

		if( verify_address(mem_address) == ADDR_VALID )
		{

            printf("valid mem write address\r\n");

            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_WritePin(MY_LED_GPIO_Port, MY_LED_Pin ,1);
            uint32_t tempBufr = 0;
            memcpy(&tempBufr,&pBuffer[7],4);
            //execute mem write
            write_status = execute_mem_write(tempBufr,mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_WritePin(MY_LED_GPIO_Port, MY_LED_Pin ,0);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}else
		{
            printf("invalid mem write address \r\n");
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
        printf("checksum fail !! \r\n");
        bootloader_send_nack();
	}

}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint32_t pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

//    for(uint32_t i = 0 ; i <len ; i+= 4)
//    {

        //Here we program the flash word to word
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,mem_address,pBuffer);
//        mem_address += 4;
//    }

    HAL_FLASH_Lock();

    return status;
}
