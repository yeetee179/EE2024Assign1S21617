#include "LPC17xx.h"
#include "stdio.h"

// EE2024 Assignment 1 skeleton
// (C) CK Tham, ECE, NUS, 2017

// PID controller written in assembly language
extern int pid_ctrl(int en, int st);
//en and st are arguments, to pass arguments from c to assembly, arg1 is stored to R0, arg2 is stored
// in R1

uint32_t usTicks=0;

// This function is called every 1us
void SysTick_Handler(void)
{
    usTicks++;
}

// Plant or System under control
// a and b are parameters of the plant
double plant(double u, unsigned start, double a, double b)
{
    static double x1, x2, x3, x4, y;

    if (start)
	{
        x1 = x2 = x3 = x4 = 0.0;
    }

    x4 = x4 + b*(u-x4);
    x3 = x3 + b*(x4-x3);
    x2 = x2 + b*(x3-x2);
    x1 = x1 + b*(x2-x1);
    y = x1;
    return(y);
}

// PID Controller written in C
// This function takes in the error and a start flag and returns the control signal
// The start flag should be 1 the first time this function is called
double PIDcontrol(double en, unsigned start)
{
    static int Kp=25, Ki=10,  Kd=80, sn, enOld, un;

    if (start)
    //at the start sn and enold needs to be zero, afterwards it will have other values
    {
        sn = enOld = 0.0;
    }
    sn = sn + en;
    if (sn>950000000) sn=950000000;
    else if (sn<-950000000) sn=-950000000;
    un = Kp*en + Ki*sn + Kd*(en-enOld);
    enOld = en;
    return(un);
}

int main(void)
{
    int i, startTicks, stopTicks;
    unsigned int st;
    double sp, y, e, u;

	// SystemTick clock configuration
	SysTick_Config(SystemCoreClock / 1000000);  // every 1us
////////////////////////////////////////////////////////////////////////////////////////////////////////
////  ASM version
//	sp = 1.0;
//	u = 0.0;
//	startTicks = usTicks;
//    for (i=0; i<50; i++)
//    {
//        if (i==0) st=1; else st=0;
//
//        y = plant(u,st,-0.8,0.2); // Do NOT change the plant parameters
//        e = sp - y;
//
//        //  Call the assembly language function pid_ctrl() here, and set control input
//
//
////        u = extern int pid_ctrl(int e, int st);
//
//
//
//       	printf("%lf\n",e);
//    }
//    stopTicks = usTicks;
//    printf("Time taken (ASM version): %ld microseconds\n",(stopTicks-startTicks));
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  C version
    sp = 1.0;
    u = 0.0;
    startTicks = usTicks;
    for (i=0; i<50; i++)
    {
        if (i==0) st=1; else st=0;

        y = plant(u,st,-0.8,0.2); // Do NOT change the plant parameters
        e = sp - y;



       	//        scale e up to whole number
       	        int e_scaled = e*1000000;
       	       	printf("e %ld\n",e_scaled);

       	        // PID controller written in C
       	//        u = PIDcontrol(e, st);
       	       	int u_scaled;
       	       	u_scaled = PIDcontrol(e_scaled, st);
       	       	printf("u %ld\n",u_scaled);

       	printf("after %lf\n",e);
    }
    stopTicks = usTicks;
    printf("Time taken (C version): %ld microseconds\n",(stopTicks-startTicks));

    // Enter an infinite loop, just incrementing a counter
	// This is for convenience to allow registers, variables and memory locations to be inspected at the end
	volatile static int loop = 0;
	while (1) {
		loop++;
	}
}
