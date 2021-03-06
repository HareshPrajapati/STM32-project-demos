#include "main.h"

int main()
{
     /*----------------------------- Ask Menu implementation----------------------------------------*/
    printf("\n\n +==========================================+");
    printf("\n |           STM32L100C  BootLoader v1           |");
    printf("\n +==========================================+\n");


    Serial_Port_Configuration();

    while(1)
    {
#if 1
        printf("\n\n +==========================================+");
        printf("\n |                   Menu                   |");
        printf("\n +==========================================+\n");
#endif
        printf("\n\n   Which BL command do you want to send ??\n");
        printf("\n   BL_GET_VER                     --> 1");
        printf("\n   BL_GET_HLP                     --> 2");
        printf("\n   BL_GET_CID                     --> 3");
        printf("\n   BL_GET_RDP_STATUS              --> 4");
        printf("\n   BL_GO_TO_ADDR                  --> 5");
        printf("\n   BL_FLASH_ERASE                 --> 6");
        printf("\n   BL_MEM_WRITE                   --> 7");
        printf("\n   MENU_EXIT                      --> 0");

        printf("\n\n   Type the command code here :");

        uint32_t command_code;
        scanf(" %d",&command_code);

        decode_menu_command_code(command_code);

#if 0
        printf("\n\n   Do you want to continue(y/n) ?:");
        uint8_t proceed = 0;
        scanf(" %c",&proceed);
        proceed -= 'y';
        if ( proceed)
        {
            printf("\n  ****** Thank you ! Exiting ******\n");
            break;
        }
#endif
        printf("\n\n   Press any key to continue  :");
        uint8_t ch = getch();
        purge_serial_port();
}
}
