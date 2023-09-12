/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>
#include <math.h>

float Gamma ;
float gamma1,gamma2;
float gamma01 = 90*M_PI/180,gamma02 = 180*M_PI/180;

float h ;
float l = 58;
float w = 0;

float h,v;
float Vx1,Vy1;

float V1,V2;
float GocLaBan;
float Angle1,Angle2;

void ocelotMode(float u, float v, float r, float Field)
{
    h = sqrt(pow(l,2)+pow(w,2))/2;
    Gamma = atan2(u,v) + Field*M_PI/180 ;
    gamma1 = Gamma + gamma01;
    gamma2 = Gamma + gamma02;
    
    Vx1 = v - h*r*sin(gamma1);
    Vy1 = u - h*r*cos(gamma1);
    
    Angle1 = atan2(Vy1,Vx1)*180/M_PI;
    V1 = sqrt(pow(Vx1,2)+pow(Vy1,2));
    
    printf(" Angle1 : %f \n",Angle1);
    printf(" Vel : %f \n",V1);
    printf(" Velx : %f \n",Vx1);
    printf(" Vely : %f \n",Vy1);
    
}
int main()
{
    ocelotMode(1,0,0.1,90);

    return 0;
}
