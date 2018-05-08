/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>

uint8 serial_buffer[3];
int sbuf_i = 0;
uint8 new_data = 0;
uint8 theta1 = 0, theta2 = 0, dtheta1 = 0, dtheta2 = 0;
char s_print[1024];

CY_ISR(RX_INT)
{
    uint8 data = UART_ReadRxData();
    if(sbuf_i == 0 && (data & ~1)) return; // we are not in sync, so wait
    serial_buffer[sbuf_i++] = data;
    if(sbuf_i == 3){
        sbuf_i = 0;
        new_data = 1;
        if(serial_buffer[0] == 0){
            theta1 = serial_buffer[1];
            dtheta1 = serial_buffer[2];
        } else {
            theta2 = serial_buffer[1];
            dtheta2 = serial_buffer[2];
        }
    }
}

void update(){
    LCD_ClearDisplay();
    sprintf(s_print, "t1=%d dt1=%d", theta1, dtheta1);
    LCD_PrintString(s_print);
}

int main()
{	
	LCD_Start();					    // initialize lcd
	LCD_ClearDisplay();
    
    CyGlobalIntEnable;
    rx_int_StartEx(RX_INT);             // start RX interrupt (look for CY_ISR with RX_INT address)
                                        // for code that writes received bytes to LCD.

    UART_Start();                       // initialize UART
    UART_ClearRxBuffer();

    for(;;)
    {
        if(new_data){
            new_data = 0;
            update();
        }
    }
}

/* [] END OF FILE */
