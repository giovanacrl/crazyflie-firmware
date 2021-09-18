#include "mbed.h"
#include "crazyflie.h"

//Declare mixer
Mixer mixer;

//Main program
int main ()
{
    //Arm mixer
    mixer.arm();
    // Actuate motor with 70% mg total thrust force (N) and zero torques
    mixer.actuate(0.7*m*g,0,0,0);
    wait (2);
    // Turn off all motors
    // Disarm mixer
    mixer.disarm();
    wait(2);
    mixer.arm();
    mixer.actuate(0.7*m*g,0,0,0);
    wait(2);
    mixer.disarm();
    // End of program
    while (true)
    {
    }
}
