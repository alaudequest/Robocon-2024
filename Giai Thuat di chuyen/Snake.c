/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>
#include <math.h>

float H ;
float Lenght = 58;
float Width = 0;
float Gamma ;
float gamma1,gamma2,gamma3,gamma4 ;
float gamma01 = 0*M_PI/180,gamma02 = 90*M_PI/180;
float gamma03 = 180*M_PI/180,gamma04 = 270*M_PI/180;

float Rcp,Phicl = 0.785;
float R1,R2,R3,R4;
float Phi1,Phi2,Phi3,Phi4;
float velFactor1,velFactor2,velFactor3,velFactor4;

float Angle1,Angle2,Angle3,Angle4;

double max(double a,double b,double c,double d)
{
	double max;
	max = a;
	if (b>=max)
	{
		max = b;
	}
	if (c>=max)
	{
	    max = c;
	}
    if (d>=max)
	{
	    max = d;
	}
	
	return max;
}

void SnakeTurn(double u,double v)
{
    H = sqrt(pow(Lenght,2)+pow(Width,2))/2;
    Gamma = atan2(u,v) ;
    gamma1 = Gamma + gamma01;
    gamma2 = Gamma + gamma02;
    gamma3 = Gamma + gamma03;
    gamma4 = Gamma + gamma04;
    
    Rcp = H/tan(Phicl);
    
    R1 = sqrt(pow(H,2)*pow(cos(gamma1),2)+pow((Rcp-H*sin(gamma1)),2));
    R2 = sqrt(pow(H,2)*pow(cos(gamma2),2)+pow((Rcp-H*sin(gamma2)),2));
    R3 = sqrt(pow(H,2)*pow(cos(gamma3),2)+pow((Rcp-H*sin(gamma3)),2));
    R4 = sqrt(pow(H,2)*pow(cos(gamma4),2)+pow((Rcp-H*sin(gamma4)),2));
    
    Phi1 = ((Phicl)*asin(H*cos(gamma1)/R1));
    Phi2 = ((Phicl)*asin(H*cos(gamma2)/R2));
    Phi3 = ((Phicl)*asin(H*cos(gamma3)/R3));
    Phi4 = ((Phicl)*asin(H*cos(gamma4)/R4));
    
    velFactor1 = R1/max(R1, R2, R3, R4);
    velFactor2 = R2/max(R1, R2, R3, R4);
    velFactor3 = R3/max(R1, R2, R3, R4);
    velFactor4 = R4/max(R1, R2, R3, R4);
    
    Angle1 = Phi1*180/M_PI;
    Angle2 = Phi2*180/M_PI;
    Angle3 = Phi3*180/M_PI;
    Angle4 = Phi4*180/M_PI;
    
    printf("R1 = %f \n",R1);
    printf("R2 = %f \n",R2);
    printf("R3 = %f \n",R3);
    printf("R4 = %f \n",R4);
    
    printf("v1 = %f \n",velFactor1);
    printf("v2 = %f \n",velFactor2);
    printf("v3 = %f \n",velFactor3);
    printf("v4 = %f \n",velFactor4);
    
    printf("Angle1 = %f \n",Angle1);
    printf("Angle2 = %f \n",Angle2);
    printf("Angle3 = %f \n",Angle3);
    printf("Angle4 = %f \n",Angle4);
    
}

int main()
{
    SnakeTurn(1,0);   
    
    return 0;
}
