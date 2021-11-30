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
const float wc = 1; 
const float alpha = (wc*dt)/(1+wc*dt);


//attitude estimator and controller com Ts = 0,3s e OS  = 0,5%
const float Ts = 0.3;    
const float OS = 0.005 ;
const float zeta = abs(log(OS))/sqrt(pow(log(OS),2) +  pow(pi,2));
const float wn = 4/(zeta*Ts);
const float kp_att = pow(wn,2);
const float kd_att = 2*zeta*wn;

const float Ts2 = 0.6; 
const float OS2 = 0.005 ;
const float zeta2 = abs(log(OS2))/sqrt(pow(log(OS2),2) +  pow(pi,2));
const float wn2 = 4/(zeta2*Ts2);
const float kp_att2 = pow(wn2,2);
const float kd_att2 = 2*zeta2*wn2;

//vertical estimator and controller 
const float dt_range = 0.05;
const float wcc = 10;   
const float zetaa = sqrt(2)/2;
const float l1 = pow(wcc,2);
const float l2 = 2*zetaa*wcc;

const float Ts_ver = 2;
const float OS_ver = 0.005 ;
const float zeta_ver = abs(log(OS_ver))/sqrt(pow(log(OS_ver),2) +  pow(pi,2));
const float wn_ver = 4/(zeta_ver*Ts_ver);
const float kp_ver = pow(wn_ver,2);
const float kd_ver = 2 * zeta_ver * wn_ver;
//const float kp_ver = 5.8567;
//const float kd_ver = 3.4225;

//horizontal estimator
const float gamma = 42.0;               // º
const float resolution = 420.0;         // px
const float sigma = (1.0/dt)*(2.0*tan((gamma*pi/180.0)/2.0))/resolution;
const float l3 = 50; //wc

const float Ts_hor = 2;
const float OS_hor = 0.005 ;
const float zeta_hor = abs(log(OS_hor))/sqrt(pow(log(OS_hor),2) +  pow(pi,2));
const float wn_hor = 4/(zeta_hor*Ts_hor);
const float kp_hor = pow(wn_hor,2);
const float kd_hor = 2*zeta_hor*wn_hor;

#endif