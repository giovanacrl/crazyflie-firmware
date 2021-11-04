# include "vertical_estimator.h"

// Class constructor
VerticalEstimator::VerticalEstimator():range(E_SDA,E_SCL)
{
    z = 0.0;
    w = 0.0;
}

// Initialize class
void VerticalEstimator::init()
{
    range.init();
}

// Predict vertical position and velocity from model
void VerticalEstimator::predict(float f_t)
{
    z = z + w * dt_range; //3.4.2
    if (z> 0.05)
    {
        w = w + ((1/m)*f_t - g) * dt_range; //3.4.2
    }  
}

// Correct vertical position and velocity with measurement
void VerticalEstimator::correct(float phi,float theta)
{
    range.read();
    if (range.d < 2.0)
    {
        float z_m = range.d * cos(phi) * cos(theta);
        z = z + l2 * dt_range * (z_m - z); //3.4.2
        w = w + l1 * dt_range * (z_m - z); //3.4.2
       
        
    }
}