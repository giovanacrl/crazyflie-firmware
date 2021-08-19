#include "mbed.h"
#include "crazyflie.h"

// Define all LEDs as digital output objects
DigitalOut ledrr(LED_RED_R,!false);
DigitalOut ledrl(LED_RED_L,!false);
DigitalOut ledbl(LED_BLUE_L,false); //perguntar se ele é oposto dos outros
DigitalOut ledgl(LED_GREEN_L,!false);
DigitalOut ledgr(LED_GREEN_R,!false);

// Define all motors as PWM objects
PwmOut motor_1(MOTOR1);
PwmOut motor_2(MOTOR2);
PwmOut motor_3(MOTOR3);
PwmOut motor_4(MOTOR4);
// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)

    // Turn on red LEDs indicating motors are armed
    
    // Test all motors with different frequencies (to make different noises)

    // Turn off red LEDs indicating motors are disarmed
    
    // Blink green LEDs indicating end of program
    while(true)
    {
        ledbl = !ledbl;
        wait(5);
        ledbl = !ledbl;
        wait(0.5);

        ledrl = !ledrl;
        ledrr = !ledrr;

        motor_2.period(1.0/1000);
        motor_2 =0.2;
        wait(1);
        motor_2.period(1.0/800);
        motor_2 =0.2;
        wait(1);
        motor_2.period(1.0/600);
        motor_2 =0.2;
        wait(1);
        motor_2.period(1.0/400);
        motor_2 =0.2;
        wait(1);

        ledrl = !ledrl;
        ledrr = !ledrr;
        motor_2 = 0.0;

        ledgl = !ledgl;
        ledgr = !ledgr;
        wait(0.5);
        ledgl = !ledgl;
        ledgr = !ledgr;
        wait(0.5);
    }
}
