#include "mbed.h"
#include "crazyflie.h"
#include <cmath>

// Define motor 1 as PWM output object
PwmOut motor(MOTOR1);

// Set PWM frequency to 500 Hz
//motor.period(1.0/500.0);

//Converts desired angular velocity (rad/s) to PWM signal (%)
float control_motor (float omega_r)
{
    float pwm;
    pwm = 1*omega_r;
    //pwm = 2.479*pow(10, -7)*pow(omega_r,2) - 0.0004551*omega_r;
    return pwm;
}

// Main programm
int main ()
{   
    motor.period(1.0/500.0);
    // Turn on motor 1 with 1.000 rad/s for 0.5 s
    motor = control_motor(1000.0);
    wait (0.5) ;
    // Turn off motor 1
    motor = 0.0;
    // End of program
    while ( true )
    {

    }
}


