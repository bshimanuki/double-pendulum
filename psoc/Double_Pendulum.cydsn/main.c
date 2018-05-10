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
float l = 7 / 2.54 / 100; // meters

CY_ISR(RX_INT)
{
    uint8 data = UART_ReadRxData();
    if(data == 0xff && sbuf_i == 1 && serial_buffer[0] != 0xff){
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

uint8 rescale(float x, float limit){
    float x_rescaled = x / limit * 0x80 + 0x80;
    uint8 value;
    if(x >= limit) value = 0xff;
    else if(x < -limit) value = 0x00;
    else value = x_rescaled;
    return value;
}

float E(float q0, float q1){
    float K = 0.5 * m * pow(l*q1, 2);
    float U = -m * g * l * cos(q0);
    return K + U;
}

float lqr(float q0bar, float q1){
    float k0 = 20;
    float k1 = 5;
    float tau = -k0 * q0bar + -k1 * q1;
    return tau;
}

float swingup(float q0, float q1){
    float k = 10;
    float q0_shift = fmod(q0 + M_PI, 2*M_PI) - M_PI;
    /*
    if(fabs(q0_shift) < 0.2 && fabs(q1) < 1){
        if(q1 < 0) q1 = -1;
        else q1 = 1;
    }
    */

    float tau = k * q1 * (E(q0, q1) - E(M_PI, 0));
    if(fabs(q1) < 0.5){
        if(fabs(q0_shift) < 0.3){
            // keep moving in same direction
            if(q1 < 0) tau += -5;
            else tau += 5;
        } else {
            // help gravity
            // if(q0_shift < 0) tau += 5;
            // else tau += -5;
        }
    }    return tau;
}

int in_roc(float q0bar, float q1){
    return fabs(q0bar) < 1;
}

void update(){
    uint8 _theta1 = theta1;
    int8 _dtheta1 = dtheta1;
    float q0 = _theta1 * 2*M_PI / SEGMENTS;
    float q0bar = q0 - M_PI;
    float q0_shift = fmod(q0 + M_PI, 2*M_PI) - M_PI;
    float q1;
    if(_dtheta1) q1 = 1 / (_dtheta1 / 225.) * (2*M_PI / SEGMENTS); // 225 = 11.0592e6/12/256/16
    else {
        // give q1 a sign
        if(fabs(q0_shift) < 0.2) q1 = 1e-2;
        else{
            if(q0_shift < 0) q1 = 1e-2;
            else q1 = -1e-2;
        }
    }

    float tau;
    if(in_roc(q0bar, q1)){
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
    DAC_u_SetValue(count);
    DAC_Debug_SetValue(rescale(E(q0, q1) - E(M_PI, 0), 3));

    DAC_Theta1_SetValue((2*_theta1) ^ 0x80);
    DAC_Dtheta1_SetValue(rescale(_dtheta1, 20));

    LCD_ClearDisplay();
    sprintf(s_print, "%d t=%d dt=%d", count, _theta1, _dtheta1);
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
    
    DAC_Theta1_Start();
    DAC_Dtheta1_Start();
    DAC_u_Start();
    DAC_Debug_Start();
    
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
