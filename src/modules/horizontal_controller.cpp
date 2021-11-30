#include "horizontal_controller.h"

//Class constructor
HorizontalController::HorizontalController()
{
    phi_r = 0.0;
    theta_r = 0.0;
    ax_r = 0.0;
    ay_r = 0.0;

}

//Control reference roll and pitch angles (rad) given reference positions (m) and currentn positions (m) and velocities (m/s)
void HorizontalController::control(float x_r, float y_r, float x, float y, float u, float v)
{
    //Aceleração x_r
    ax_r = control_siso(x_r, x, u, kp_hor, kd_hor);
    //Aceleração y_r
    ay_r = control_siso(y_r, y, v, kp_hor, kd_hor);

    theta_r = (1/g) * ax_r;
    phi_r = -(1/g) * ay_r;
}

//Control acceleration given reference position (m) and current position (m) and velocity (m/s) with given controller gains
float HorizontalController::control_siso(float pos_r, float pos, float vel, float kp, float kd)
{
    //float vx_r = kp * (pos_r - pos);
    //return float (kd * (vx_r - vel));
    return (float(kp * (pos_r - pos) + kd * (0.0 - vel)));
}