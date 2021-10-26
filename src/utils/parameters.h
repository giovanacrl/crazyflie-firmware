#ifndef parameters_h
#define parameters_h

#include <cmath>

// Physical constants
const float pi = 3.1416;
const float g = 9.81;       // m/s^2

// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 29.0e-6; // kg.m^2
const float l = 33.0e-3;    // m

// Constantes da curva fit pwm = a2*w^2 + a1* w para relacionar a velocidade angular com pwm
const float a2 = 1.16e-7;
const float a1 = 4.488e-12;

// Constante de sustentação kl 
const float kl = 1.734e-8;

// Constante de arrasto kd
const float kd = 1.482e-10;

const float dt = 0.002;
const float wc = 10;
const float alpha = (wc*dt)/(1+wc*dt);

//attitude estimator and controller com Ts = 0,3s e OS  = 0,5%
const float Ts = 0.3;
const float OS = 0.005 ;
const float zeta = abs(log(OS))/sqrt(pow(log(OS),2) +  pow(pi,2));
const float wn = 4/(zeta*Ts);
const float kp_att = pow(wn,2);
const float kd_att = 2*zeta*wn;


#endif