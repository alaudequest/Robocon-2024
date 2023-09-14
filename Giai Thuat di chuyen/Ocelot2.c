/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/
#include <stdio.h>
#include <math.h>

float Gamma ;
float gamma1,gamma2;
float gamma01 = 0*M_PI/180,gamma02 = 180*M_PI/180;

float h ;
float l = 58;
float w = 0;

float h,v;
float Vx1,Vy1;
float Vx2,Vy2;

float V1,V2;
float GocLaBan;
float Angle1,Angle2;

void ocelotMode(float u, float v, float r, float Field)
{
    h = sqrt(pow(l,2)+pow(w,2))/2;
    if((u == 0)&&(v == 0))
    {
        Vx1 = v;
        Vy1 = u - h*r;
        
        Vx2 = v;
        Vy2 = u + h*r;
    }
    else
    {
        Gamma = atan2(u,v) + Field*M_PI/180 ;
        gamma1 = Gamma + gamma01;
        gamma2 = Gamma + gamma02;
        
        Vx1 = v*sin(Gamma) - h*r*sin(gamma1);
        Vy1 = u*cos(Gamma) - h*r*cos(gamma1);
        
        Vx2 = v*sin(Gamma) - h*r*sin(gamma2);
        Vy2 = u*cos(Gamma) - h*r*cos(gamma2);
    }
        
    Angle1 = atan2(Vy1,Vx1)*180/M_PI;
    Angle2 = atan2(Vy2,Vx2)*180/M_PI;
    
    V1 = sqrt(pow(Vx1,2)+pow(Vy1,2));
    V2 = sqrt(pow(Vx2,2)+pow(Vy2,2));
    
    
    
    printf(" Field : %f \n",Field);
    printf(" Angle1 : %f \n",Angle1);
    printf(" Angle2 : %f \n",Angle2);
    
    printf(" Vel1 : %f \n",V1);
    printf(" Vel2 : %f \n",V2);
    
    printf(" Velx1 : %f \n",Vx1);
    printf(" Vely1 : %f \n",Vy1);
    
    printf(" Velx2 : %f \n",Vx2);
    printf(" Vely2 : %f \n",Vy2);
    printf("//////////////////////////////////////////////////// \n");
}
int i = -90;
int main()
{
    while (i<180){
        
    ocelotMode(0,0,0.1,i);
    i+=5;
    }

    return 0;
}