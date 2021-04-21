

/* This file implements logic to decode the user input , prepare and send the bootloader command packet
 * over the serial port. This file is common across win/linux/mac
 */

#include "main.h"

//Decode the Bootloader command selection by the user
void decode_menu_command_code(uint32_t command_code)
{

    unsigned char data_buf[255];
    uint32_t crc32=0; int ret_value=0;

    switch(command_code)
    {
    case 0:
        printf("\n  Exiting...!");
        exit(0);

    case 1:
        printf("\n   Command == > BL_GET_VER");

        data_buf[0] = COMMAND_BL_GET_VER_LEN-1;
        data_buf[1] = COMMAND_BL_GET_VER;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_VER_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_VER_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 2:
        printf("\n   Command == > BL_GET_HLP");

        data_buf[0] = COMMAND_BL_GET_HELP_LEN-1;
        data_buf[1] = COMMAND_BL_GET_HELP;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_HELP_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_HELP_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 3:
        printf("\n   Command == > BL_GET_CID");

        data_buf[0] = COMMAND_BL_GET_CID_LEN-1;
        data_buf[1] = COMMAND_BL_GET_CID;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_CID_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_CID_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 4:
        printf("\n   Command == > BL_GET_RDP_STATUS");

        data_buf[0] = COMMAND_BL_GET_RDP_STATUS_LEN-1;
        data_buf[1] = COMMAND_BL_GET_RDP_STATUS;
        crc32       = get_crc(data_buf,COMMAND_BL_GET_RDP_STATUS_LEN-4);
        data_buf[2] = word_to_byte(crc32,1,1);
        data_buf[3] = word_to_byte(crc32,2,1);
        data_buf[4] = word_to_byte(crc32,3,1);
        data_buf[5] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GET_RDP_STATUS_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 5:
        printf("\n   Command == > BL_GO_TO_ADDR");
        printf("\n\n   Enter Address here :");
        uint32_t go_address;
        scanf(" %x",&go_address);

        data_buf[0] = COMMAND_BL_GO_TO_ADDR_LEN-1;
        data_buf[1] = COMMAND_BL_GO_TO_ADDR;
#if 0
        data_buf[2] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,1,1);
        data_buf[3] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,2,1);
        data_buf[4] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,3,1);
        data_buf[5] = word_to_byte(RESET_HANDLER_ADDR_OF_APP,4,1);
#endif
        data_buf[2] = word_to_byte(go_address,1,1);
        data_buf[3] = word_to_byte(go_address,2,1);
        data_buf[4] = word_to_byte(go_address,3,1);
        data_buf[5] = word_to_byte(go_address,4,1);
        crc32       = get_crc(data_buf,COMMAND_BL_GO_TO_ADDR_LEN-4);
        data_buf[6] = word_to_byte(crc32,1,1);
        data_buf[7] = word_to_byte(crc32,2,1);
        data_buf[8] = word_to_byte(crc32,3,1);
        data_buf[9] = word_to_byte(crc32,4,1);

        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_GO_TO_ADDR_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 6:
        printf("\n   Command == > BL_FLASH_ERASE");

        data_buf[0] = COMMAND_BL_FLASH_ERASE_LEN-1;
        data_buf[1] = COMMAND_BL_FLASH_ERASE;
        uint32_t page_num,nsec , address_of_page;

        printf("\n  Enter page number(0-1023) here :");
        scanf(" %d",&page_num);
        printf("\n Enter the address of %d :",page_num);
        scanf(" %x",&address_of_page);
        printf("\n  Enter number of page to erase(max 1023) here :");
        scanf(" %d",&nsec);


        data_buf[2]= page_num;
        data_buf[3]= nsec;

        crc32       = get_crc(data_buf,COMMAND_BL_FLASH_ERASE_LEN-8);
        data_buf[4] = word_to_byte(crc32,1,1);
        data_buf[5] = word_to_byte(crc32,2,1);
        data_buf[6] = word_to_byte(crc32,3,1);
        data_buf[7] = word_to_byte(crc32,4,1);
        data_buf[8] = word_to_byte(address_of_page,1,1);
        data_buf[9] = word_to_byte(address_of_page,2,1);
        data_buf[10] = word_to_byte(address_of_page,3,1);
        data_buf[11] = word_to_byte(address_of_page,4,1);



        Write_to_serial_port(&data_buf[0],1);
        Write_to_serial_port(&data_buf[1],COMMAND_BL_FLASH_ERASE_LEN-1);

        ret_value = read_bootloader_reply(data_buf[1]);

        break;

    case 7:
        printf("\n   Command == > BL_MEM_WRITE");
        uint32_t bytes_remaining=0;
        uint32_t t_len_of_file=0;
        uint32_t bytes_so_far_sent = 0,len_to_read=0;
        uint32_t base_mem_address=0;

        data_buf[1] = COMMAND_BL_MEM_WRITE;

        //First get the total number of bytes in the .bin file.
        t_len_of_file =calc_file_len();

        //keep opening the file
        open_the_file();

        bytes_remaining = t_len_of_file - bytes_so_far_sent;
        printf(" \r\n byte remaining = %u \r\n",bytes_remaining);

#if 0
        //Here you should  get it from user input . but
        //for testing purpose just hard coded.
        base_mem_address = USE_APP_FLASH_BASE_ADDR;
#endif
        printf("\n\n   Enter the memory write address here :");
        scanf(" %x",&base_mem_address);

        while(bytes_remaining)
        {

            if(bytes_remaining >= 4)
            {
                len_to_read = 4;
            }else
            {
                len_to_read = bytes_remaining;
            }

            //get the bytes in to buffer by reading file
            read_the_file(&data_buf[7],len_to_read);

            printf("\n   base mem address = %#.8x\n",base_mem_address);

            //populate base mem address
            data_buf[2] = word_to_byte(base_mem_address,1,1);
            data_buf[3] = word_to_byte(base_mem_address,2,1);
            data_buf[4] = word_to_byte(base_mem_address,3,1);
            data_buf[5] = word_to_byte(base_mem_address,4,1);

            data_buf[6] = len_to_read;

            /* 1 byte len + 1 byte command code + 4 byte mem base address
             * 1 byte payload len + len_to_read is amount of bytes read from file + 4 byte CRC
             */
            uint32_t mem_write_cmd_total_len = COMMAND_BL_MEM_WRITE_LEN(len_to_read);
            printf("mem_write_cmd_total_len = %u \r\n",mem_write_cmd_total_len);

            //first field is "len_to_follow"
            data_buf[0] =mem_write_cmd_total_len-1;

            crc32       = get_crc(&data_buf[0],mem_write_cmd_total_len-4);
            data_buf[7+len_to_read] = word_to_byte(crc32,1,1);
            data_buf[8+len_to_read] = word_to_byte(crc32,2,1);
            data_buf[9+len_to_read] = word_to_byte(crc32,3,1);
            data_buf[10+len_to_read] = word_to_byte(crc32,4,1);

            //update base mem address for the next loop
            base_mem_address+=len_to_read;

            Write_to_serial_port(&data_buf[0],1);
            Write_to_serial_port(&data_buf[1],mem_write_cmd_total_len-1);

            bytes_so_far_sent+=len_to_read;
            bytes_remaining = t_len_of_file - bytes_so_far_sent;

            printf("\n\n    bytes_so_far_sent:%d -- bytes_remaining:%d\n",bytes_so_far_sent,bytes_remaining);

           ret_value = read_bootloader_reply(data_buf[1]);

         }
        break;
    default:
        {
            printf("\n\n  Please input valid command code\n");
            return;
        }

    }

        if(ret_value == -2)
        {
            printf("\n\n   TimeOut : No response from the bootloader");
            printf("\n   Reset the board and Try Again !\n");
            return;
        }


}
