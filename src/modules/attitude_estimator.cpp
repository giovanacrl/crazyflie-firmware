#include "attitude_estimator.h"

//Class constructor
AttitudeEstimator::AttitudeEstimator() : imu(IMU_SDA,IMU_SCL), ledrl(LED_RED_L,!false)
{
    phi = 0.0;
    theta = 0.0;
    psi = 0.0;
    p = 0.0;
    q = 0.0;
    r = 0.0;
    p_bias = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
}

//Initilize class 
void AttitudeEstimator::init()
{
    imu.init();
    for (int i=0; i<500; i++)
    {
        imu.read();
        gx = gx + imu.gx;
        gy = gy + imu.gy;
        gz = gz + imu.gz;
        wait(dt);
    }
    p_bias = (1/500)*gx;
    q_bias = (1/500)*gy;
    r_bias = (1/500)*gz;

    ledrl = !ledrl;
    wait(1);
    ledrl = !ledrl;

}

//Estimate Euler angles (rad) and angular velocities (rad/s)
void AttitudeEstimator::estimate()
{
    imu.read();

    p = imu.gx - p_bias;
    q = imu.gy - q_bias;
    r = imu.gz - r_bias;

//LINEAR
    //float phi_g = phi + p*dt;
    //float theta_g = theta + q*dt;
    //float psi_g = psi + r*dt;

    //float phi_a = atan2(-imu.ay,-imu.az);
    //float theta_a = atan2(imu.ax,-imu.az);
    


//NÃO LINEAR
    float phi_g = phi + (p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r)*dt;
    float theta_g = theta + (cos(phi)*q - sin(phi)*r)*dt;
    float psi_g = psi + (q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta))*dt;

    float phi_a = atan2(-imu.ay,-imu.az);
    float theta_a = atan2(imu.ax,(-((imu.az>0)-(imu.az<0))*sqrt(pow(imu.ay, 2)+pow(imu.az, 2))));
    


    phi = (1-alpha)*phi_g + alpha*phi_a;
    theta = (1-alpha)*theta_g + alpha*theta_a;
    psi = psi_g;


}