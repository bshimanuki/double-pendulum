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
#include <math.h>

#define SEGMENTS 128

uint8 serial_buffer[2];
int sbuf_i = 0;
uint8 new_data = 0;
uint8 theta1 = 0, theta2 = 0;
int8 dtheta1 = 0, dtheta2 = 0;
char s_print[1024];

// model parameters
float g = 9.8;
float m = 0.1; // kg
float l = 7 / 2.54 * 100; // meters

CY_ISR(RX_INT)
{
    uint8 data = UART_ReadRxData();
    if(data == 0xff && sbuf_i == 1){
        // 0xff is not possible for second byte so we must not be in sync
        serial_buffer[0] = data;
        return;
    }
    serial_buffer[sbuf_i++] = data;
    if(sbuf_i == 2){
        sbuf_i = 0;
        new_data = 1;
        if(serial_buffer[0] == 0xff && serial_buffer[1] == 0xff){ // reset code
            theta1 = 0;
            theta2 = 0;
            dtheta1 = 0;
            dtheta2 = 0;
        } else {
            if(!(serial_buffer[0] & 0x80)){
                theta1 = serial_buffer[0] & 0x7f;
                dtheta1 = (int8) serial_buffer[1];
            } else {
                theta2 = serial_buffer[0] & 0x7f;
                dtheta2 = (int8) serial_buffer[1];
            }
        }
    }
}

float E(float q0, float q1){
    K = 0.5 * m * pow(l*q1, 2);
    U = -m * g * l * cos(q0);
    return K + U;
}

float lqr(float q0, float q1){
    float k0 = 8;
    float k1 = 0.1;
    float tau = -k0 * q0 + -k1 * q1;
    return tau;
}

float swingup(float q0, float q1){
    float k = 1;
    if(abs(q0) < 0.2 && abs(q1) < 1){
        if(q1 < 0) q1 = -1;
        else q1 = 1;
    }
    float tau = k * q1 * (E(q0, q1) - E(M_PI, 0));
    return tau;
}

bool in_roc(float q0, float q1){
    return abs(q0) < 0.3;
}

void update(){
    uint8 _theta1 = theta1;
    int8 _dtheta1 = dtheta1;
    float q0 = _theta1;
    float q0bar = q0 * 2*M_PI / SEGMENTS - M_PI;
    float q1;
    if(_dtheta1) q1 = 1 / (_dtheta1 / 225.) * (2*M_PI / SEGMENTS); // 225 = 11.0592e6/12/256/16
    else q1 = 0;

    float tau;
    if(in_roc(q0, q1)){
        tau = lqr(q0bar, q1);
    } else {
        tau = swingup(q0, q1);
    }

    float tau_limit = 5;
    float duty = 1./2 + tau / (2*tau_limit);
    uint8 count;
    if(duty >= 1) count = 255;
    else if(duty <= 0) count = 0;
    else count = 255 * duty;

    PWM_WriteCompare(count);

    LCD_ClearDisplay();
    sprintf(s_print, "%d t1=%d dt1=%d", count, _theta1, _dtheta1);
    LCD_PrintString(s_print);
    LCD_Position(1, 0);
    sprintf(s_print, "q0=%.2f q1=%.2f", q0, q1);
    LCD_PrintString(s_print);
}

CY_ISR(BUTTON_INT)
{
    PWM_WriteCompare(0xc0);
}

int main()
{
    Clock_Start();
    LCD_Start();                        // initialize lcd
    LCD_ClearDisplay();
    PWM_Start();
    
    CyGlobalIntEnable;
    rx_int_StartEx(RX_INT);             // start RX interrupt (look for CY_ISR with RX_INT address)
                                        // for code that writes received bytes to LCD.

    button_int_StartEx(BUTTON_INT);
    PWM_WriteCompare(0x80);

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
