#ifndef mixer_h
#define mixer_h

#include "mbed.h"
#include "crazyflie.h"

//Mixer class

class Mixer
{
    public:
    //Class constructor
        Mixer();
        //Actuate motors with desired total trust force (N) and torques (N.m)
        void arm();
        void disarm();
        void actuate(float f_t, float tau_phi, float tau_theta, float tau_psi);
    private:
        //Motors PWM outputs
        PwmOut motor_1, motor_2, motor_3, motor_4;
        DigitalOut ledgl, ledgr, ledrl, ledrr, ledbl;
        //Angular velocities (rad/s)
        float omega_r_1, omega_r_2, omega_r_3, omega_r_4;
        //Convert total thrust force (N) and torques (N.m) to angular velocities (rad/s)
        void mixer(float f_t, float tau_phi, float tau_theta, float tau_psi);
        //Convert desired angular velocity (rad/s) to PWM signal (%)
        float control_motor(float omega_r);
        bool armed;

};

#endif
