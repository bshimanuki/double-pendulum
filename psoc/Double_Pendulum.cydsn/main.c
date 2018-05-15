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

#define false 0
#define true 1
typedef int bool;

#define SEGMENTS 128

uint8 serial_buffer[2];
int sbuf_i = 0;
uint8 new_data = 0;
uint8 theta1 = 0, theta2 = 0;
int8 dtheta1 = 0, dtheta2 = 0;
char s_print[1024];

float E_top;
float q0, q0bar, q1, v0, v1;
float v0_buf[3], v1_buf[3]; // median filter
int v0_buf_i = 0, v1_buf_i = 0;

// model parameters
float g = 9.8;
float m1 = 0.1; // kg
float l1 = 5 * 2.54 / 100; // meters
float lc1 = 5 * 2.54 / 100; // meters
float m2 = 0.001; // kg
float l2 = 4 * 2.54 / 100; // meters
float lc2 = 4 * 2.54 / 100; // meters
float I1, I2;

float min(float a, float b){return a < b ? a : b;}
float max(float a, float b){return a > b ? a : b;}

CY_ISR(RX_INT)
{
    uint8 data = UART_ReadRxData();
    if(data == 0xff && sbuf_i == 1 && serial_buffer[0] != 0xff){
        // 0xff is not possible for second byte so we must not be in sync
        serial_buffer[0] = data;
        return;
    }
    serial_buffer[sbuf_i++] = data;
    if(sbuf_i == 1){
        sbuf_i = 0;
        new_data = 1;
        if(serial_buffer[0] == 0xff && serial_buffer[1] == 0xff){ // reset code
            theta1 = 0;
            theta2 = 0;
            dtheta1 = 0;
            dtheta2 = 0;
            q0 = 0;
            q1 = 0;
            v0 = 0;
            v1 = 0;
            memset(v0_buf, 0, sizeof(v0_buf));
            memset(v1_buf, 0, sizeof(v1_buf));
            v0_buf_i = 0;
            v1_buf_i = 0;
        } else {
            if(!(serial_buffer[0] & 0x80)){
                uint8 last_theta1 = theta1;
                theta1 = serial_buffer[0] & 0x7f;
                dtheta1 = (int8) serial_buffer[1];

                q0 = theta1 * 2*M_PI / SEGMENTS;
                q0bar = q0 - M_PI;
                
                /*
                if(dtheta1) v0 = 1 / (dtheta1 / (11.0592e6 / 12. / 256. / 2.)) * (2*M_PI / SEGMENTS); // 256 counts per R31JP interrupt, 2 interrupts per bit
                else {
                    // give v0 a sign
                    if(v0 < 0) v0 = 1e-2;
                    else v0 = -1e-2;
                }
                v0_buf[v0_buf_i++] = v0;
                if(v0_buf_i == 3) v0_buf_i = 0;
                v0 = v0_buf[0] + v0_buf[1] + v0_buf[2] - min(v0_buf[0], min(v0_buf[1], v0_buf[2])) - max(v0_buf[0], max(v0_buf[1], v0_buf[2]));
                */
                
                int diff = (int) theta1 - (int) last_theta1;
                if(diff < -SEGMENTS/2) diff += SEGMENTS;
                if(diff > SEGMENTS/2) diff -= SEGMENTS;
                if(v0 * diff < 0){
                    if(v0 < 0) v0 = 1e-2;
                    else v0 = -1e-2;
                } else {
                    if(diff < 0) v0 = -1;
                    else v0 = 1;
                    v0 *= 1. / ((1LL<<32) - Timer0_ReadCounter()) * 24e6 * (2*M_PI / SEGMENTS);
                }
                Timer0_WriteCounter((1LL<<32)-1);
            } else {
                uint8 last_theta2 = theta2;
                theta2 = serial_buffer[0] & 0x7f;
                dtheta2 = (int8) serial_buffer[1];

                // q1, v1 sensors are in opposite direction
                if(theta2) q1 = 2*M_PI - theta2 * 2*M_PI / SEGMENTS;
                else q1 = 0;
                /*
                if(dtheta2) v1 = -1 / (dtheta2 / (11.0592e6 / 12. / 256. / 2.)) * (2*M_PI / SEGMENTS); // 256 counts per R31JP interrupt, 2 interrupts per bit
                else {
                    // give v0 a sign
                    if(v1 < 0) v1 = 1e-2;
                    else v1 = -1e-2;
                }
                v1_buf[v1_buf_i++] = v1;
                if(v1_buf_i == 3) v1_buf_i = 0;
                v1 = v1_buf[0] + v1_buf[1] + v1_buf[2] - min(v1_buf[0], min(v1_buf[1], v1_buf[2])) - max(v1_buf[0], max(v1_buf[1], v1_buf[2]));
                */
                
                int diff = -((int) theta2 - (int) last_theta2);
                if(diff < -SEGMENTS/2) diff += SEGMENTS;
                if(diff > SEGMENTS/2) diff -= SEGMENTS;
                if(v1 * diff < 0){
                    if(v1 < 0) v1 = 1e-2;
                    else v1 = -1e-2;
                } else {
                    if(diff < 0) v1 = -1;
                    else v1 = 1;
                    v1 *= 1. / ((1LL<<32) - Timer1_ReadCounter()) * 24e6 * (2*M_PI / SEGMENTS);
                }
                Timer1_WriteCounter((1LL<<32)-1);
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

float _E(float _q0, float _q1, float _v0, float _v1){
    float K = 0.5 * (
            I1 * pow(_v0, 2)
            + I2 * pow(_v0 + _v1, 2)
            + m2 * pow(l1 * _v0, 2)
            + 2 * m2 * l1 * lc2 * cos(_q1) * _v0 * (_v0 + _v1)
            );
    float U = -m1 * g * lc1 * cos(_q0) - m2 * g * (l1 * cos(_q0) + lc2 * cos(_q0 + _q1));
    return K + U;
}
float E(){ return _E(q0, q1, v0, v1); }

float ddE_dtdu(){
    // ddv_dtdu = np.linalg.lstsq(M, B[:,0])[0]
    float denominator = I1 * I2 + m2 * pow(l1, 2)  * I2 - pow(m2 * l1 * lc2 * cos(q1), 2);
    float ddv_dtdu0 = I2 / denominator;
    float ddv_dtdu1 = (-I2 - m2 * l1 * lc2 * cos(q1)) / denominator;
    // ddE_dtdu = 1./2 * (np.dot(ddv_dtdu.T, np.dot(M, v)) + np.dot(v.T, B[:,0]))
    float ddE = 0.5 * (
            I1 * ddv_dtdu0 * v0
            + I2 * (ddv_dtdu0 + ddv_dtdu1) * (v0 + v1)
            + m2 * pow(l1, 2) * ddv_dtdu0 * v0
            + m2 * l1 * lc2 * cos(q1) * (2 * ddv_dtdu0 *v0 + ddv_dtdu0 * v1 + ddv_dtdu1 * v0)

            + v0
            );
    return ddE;
}

float lqr(){
    float k0 = 20;
    float k1 = 2;
    float tau = -k0 * q0bar + -k1 * v0;
    return tau;
}

float swingup(){
    float k = 100;
    float k_e = 1.4;
    /*
    float q0_shift = fmod(q0 + M_PI, 2*M_PI) - M_PI;
    if(fabs(q0_shift) < 0.2 && fabs(v0) < 0.3){
        if(v0 < 0) v0 = -10;
        else v0 = 10;
    }
    */

    float tau = -k * (ddE_dtdu() < 0 ? -1 : 1) * (E() - k_e*E_top);
    // if(fabs(v0) < 0.5){
        // if(fabs(q0_shift) < 0.3){
            // // keep moving in same direction
            // if(v0 < 0) tau += -5;
            // else tau += 5;
        // } else {
            // // help gravity
            // // if(q0_shift < 0) tau += 5;
            // // else tau += -5;
        // }
    // }
    return tau;
}

int in_roc(){
    return fabs(q0bar) < 0.5;
}

void update(){

    float tau;
    if(in_roc()){
        tau = lqr();
    } else {
        tau = swingup();
    }

    uint8 count = rescale(tau, 5);
    PWM_WriteCompare(count);
    DAC_u_SetValue(count);
    DAC_Debug_SetValue(rescale(E() - E_top, E_top));

    DAC_Theta1_SetValue((2*theta1) ^ 0x80);
    DAC_Dtheta1_SetValue(rescale(v0, 10));

    LCD_ClearDisplay();
    sprintf(s_print, "%d t=%d dt=%d", count, theta1, dtheta1);
    sprintf(s_print, "%d E=%.2f", count, E() - E_top);
    //LCD_PrintString(s_print);
    //LCD_Position(1, 0);
    sprintf(s_print, "q0=%.2f q1=%.2f", q0, q1);
    LCD_PrintString(s_print);
    LCD_Position(1, 0);
    sprintf(s_print, "v0=%.2f v1=%.2f", v0, v1);
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
    Timer0_Start();
    Timer1_Start();
   
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

    // initialize computed constants
    E_top = _E(M_PI, 0, 0, 0);
    I1 = m1 * pow(lc1, 2);
    I2 = m2 * pow(lc2, 2);

    update();
    for(;;)
    {
        if(new_data){
            new_data = 0;
            update();
        }
    }
}

/* [] END OF FILE */
