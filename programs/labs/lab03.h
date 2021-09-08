#include "mbed.h"
#include "crazyflie.h"
#include <cmath>

// Define all motors as PWM objects
PwmOut motor_1(MOTOR1);
PwmOut motor_2(MOTOR2);
PwmOut motor_3(MOTOR3);
PwmOut motor_4(MOTOR4);

//Define angular velocities (rad/s)
float omega_1;
float omega_2;
float omega_3;
float omega_4;



//Converts desired angular velocity (rad/s) to PWM signal (%)
float control_motor(float omega_r)
{
    float pwm;
    float a2 = 1.16e-7;
    float a1 = 4.488e-12;
    pwm = a2*pow(omega_r, 2) + a1*omega_r;
    return pwm;
}

//Converts total trust force (N) and torques (N.m) to angular velocities (rad/s)
void mixer(float f_t, float tau_phi, float tau_theta, float tau_psi)
{
    float kl = 1.734e-8;
    float kd = 1.482e-10;
    float l = 33e-3;

    float t1 = 1/(4*kl);
    float t2 = 1/(4*kl*l);
    float t3 = 1/(4*kd);

    omega_1 = sqrt(t1*f_t - t2*tau_phi - t2*tau_theta - t3*tau_psi);
    omega_2 = sqrt(t1*f_t - t2*tau_phi + t2*tau_theta + t3*tau_psi);
    omega_3 = sqrt(t1*f_t + t2*tau_phi + t2*tau_theta - t3*tau_psi);
    omega_4 = sqrt(t1*f_t + t2*tau_phi - t2*tau_theta + t3*tau_psi);
//fabs()
}

//Actuate motors with desired total trust force (N) and torques (N.m)
void actuate(float f_t, float tau_phi, float tau_theta, float tau_psi)
{
    mixer(f_t, tau_phi, tau_theta, tau_psi);
    motor_1 = control_motor(omega_1);
    motor_2 = control_motor(omega_2);
    motor_3 = control_motor(omega_3);
    motor_4 = control_motor(omega_4);
}

// Main program
int main ()
{   
    //Set all PWM frequencies to 500 Hz
    motor_1.period(1.0/500.0);
    motor_2.period(1.0/500.0);
    motor_3.period(1.0/500.0);
    motor_3.period(1.0/500.0);
    //actuate motor with 70% mg total thrust force (N) and zero torques (N.m)
    actuate(0,0,0,-0.001); //precisa incluir o arq parametro? 
    wait(5);
    //Turn off all motors 
    actuate(0,0,0,0);
    //End of program
    while ( true )
    {
    }
}


