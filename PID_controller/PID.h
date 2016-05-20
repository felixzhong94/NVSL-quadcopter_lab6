//For PID loop

class PID
{
    #define UNINIT  0
    #define IDLE	1
    #define RUNNING 2
    #define STOPPED 3
    #define ERROR   4
  
    public:
        PID(double*, double*, double*); //&setpoint, &input, &output
        void SetDt(int);
        void SetK(double, double, double);
        void Initialize();
        int  GetState();
        
    private:
        int Dt;
        int state;
        
        double kp, ki, kd;
        
        double *setpoint;
        double *input;
        
        double *output;
        double outMin, outMax;
        
        void compute();    
};
