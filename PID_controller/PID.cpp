#include "PID.h"
#include <Scheduler.h>

PID::PID(double* s, double* i, double* o){
    setpoint = s;
    input = i;
    output = o;   
} 

void PID::SetDt(int dt){Dt = dt;}
void PID::SetK(double p, double i, double d){kp = p; ki = i; kd = d;}                

void PID::Initialize()
{
  Scheduler.startLoop(compute);
}
int PID::GetState(){return state;}

void PID::compute()
{
    
}
