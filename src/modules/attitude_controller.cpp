#include "attitude_controller.h"
#include "parameters.h"


AttitudeEstimator att_est;
//Class constructor 
AttitudeController::AttitudeController()
{
    tau_phi = 0.0;
    tau_theta = 0.0;
    tau_psi = 0.0;

    aphi_r = 0.0;
    atheta_r = 0.0;
    apsi_r = 0.0;

    a_ang = 0.0;
}

//Control torque (N.m) given references angles (rad) and current angles (rad) and angular velocities (rad/s)
void AttitudeController::control(float phi_r, float theta_r, float psi_r, float phi, float theta, float psi, float p, float q, float r)
{
    //aceleração phi_r
    aphi_r = control_siso(phi_r, phi, p, kp_att, kd_att);
    //aceleração theta_r
    atheta_r = control_siso(theta_r, theta, q, kp_att, kd_att);
    //aceleração psi_r
    apsi_r = control_siso(psi_r, psi, r, kp_att, kd_att);  

    //torques
    tau_phi = I_xx * aphi_r ;
    tau_theta = I_yy * atheta_r ;
    tau_psi = I_zz * apsi_r;
}

//Control torques (N.m) given references angle (rad) and current angle (rad) and angular velocity (rad/s) with given controller gains
float AttitudeController::control_siso(float angle_r, float angle, float v_ang, float kp_att, float kd_att)
{
    //Controlador em cascata 
    return(kp_att * (angle_r - angle) + kd_att * (0 - v_ang));
    
}


