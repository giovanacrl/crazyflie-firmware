# include "mixer.h"

// Class constructor
Mixer :: Mixer () : motor_1 ( MOTOR1 ) , motor_2 ( MOTOR2 ) , motor_3 ( MOTOR3 ) , motor_4 ( MOTOR4 ), ledbl(LED_BLUE_L,false), ledgl(LED_GREEN_L,!false), ledgr(LED_GREEN_R,!false), ledrl(LED_RED_L,!false), ledrr(LED_RED_R,!false)
{
    motor_1 . period (1.0/500.0) ;
    motor_2 . period (1.0/500.0) ;
    motor_3 . period (1.0/500.0) ;
    motor_4 . period (1.0/500.0) ;
    motor_1 = 0.0;
    motor_2 = 0.0;
    motor_3 = 0.0;
    motor_4 = 0.0;
    bool armed = false;
 }

 void Mixer::arm()
 {
    armed = true;
    if (ledgl == false) {
    ledgl=true;
    ledgr=true;
    }
    ledrr = false;
    ledrl = false;
    ledbl = !ledbl;
    wait(1);
    ledbl = !ledbl;
    wait(1);
    ledbl = !ledbl;
    wait(1);
    ledbl = !ledbl;
    wait(1);
    ledbl = !ledbl; 
    wait(1);
    ledbl = !ledbl;     
 }

void Mixer::disarm()
{
    armed = false;
    if (ledrl == false) {
    ledrl=true;
    ledrr=true;
    }
    ledgr = false;
    ledgl = false;
    actuate(0, 0, 0, 0);
}

 // Actuate motors with desired total trust force (N) and torques (N.m)
 void Mixer::actuate ( float f_t , float tau_phi , float tau_theta , float tau_psi )
 {
    if (armed == true) {
    mixer (f_t , tau_phi , tau_theta , tau_psi );
    motor_1 = control_motor ( omega_r_1 );
    motor_2 = control_motor ( omega_r_2 );
    motor_3 = control_motor ( omega_r_3 );
    motor_4 = control_motor ( omega_r_4 );
    }
    else {
    mixer (0,0,0,0);
    motor_1 = control_motor ( omega_r_1 );
    motor_2 = control_motor ( omega_r_2 );
    motor_3 = control_motor ( omega_r_3 );
    motor_4 = control_motor ( omega_r_4 );
    }
 }


 // Convert total trust force (N) and torques (N.m) to angular velocities ( rad /s)
 void Mixer :: mixer ( float f_t , float tau_phi , float tau_theta , float tau_psi )
 {
    float kl = 1.734e-8;
    float kd = 1.482e-10;
    float l = 33e-3;

    float t1 = 1/(4*kl);
    float t2 = 1/(4*kl*l);
    float t3 = 1/(4*kd);

    float t_omega1 = t1*f_t - t2*tau_phi - t2*tau_theta - t3*tau_psi;
    float t_omega2 = t1*f_t - t2*tau_phi + t2*tau_theta + t3*tau_psi;
    float t_omega3 = t1*f_t + t2*tau_phi + t2*tau_theta - t3*tau_psi;
    float t_omega4 = t1*f_t + t2*tau_phi - t2*tau_theta + t3*tau_psi;

    if (t_omega1 < 0) {
    float t_omega1 = 0;
    }
    if (t_omega2 < 0) {
    float t_omega2 = 0;
    }
    if (t_omega3 < 0) {
    float t_omega3 = 0;
    }
    if (t_omega4 < 0) {
    float t_omega4 = 0;
    }

    omega_r_1 = sqrt(t_omega1);
    omega_r_2 = sqrt(t_omega2);
    omega_r_3 = sqrt(t_omega3);
    omega_r_4 = sqrt(t_omega4);
 }

 // Convert desired angular velocity ( rad /s) to PWM signal (%)
 float Mixer :: control_motor ( float omega_r )
 {
    float pwm;
    float a2 = 1.16e-7;
    float a1 = 4.488e-12;
    pwm = a2*pow(omega_r, 2) + a1*omega_r;
    return pwm; //rever isso
 }
