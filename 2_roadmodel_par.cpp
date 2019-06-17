#include <ros/ros.h>
#include "egolane/datax.h"

#include <sstream>
#include<iostream>
#include<iomanip>
#include <math.h>
#include <string>
#include <cmath>

using namespace std;
egolane::datax center;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<egolane::datax>("parameters", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("messagePoints", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const egolane::datax& input)
  {
    egolane::datax parameter;
    
	
	center = input;	
	
	ROS_INFO("I heard X: [%f, %f, %f, %f]", center.x0, center.x1, center.x2, center.x3);
	ROS_INFO("I heard Y: [%f, %f, %f, %f]", center.y0, center.y1, center.y2, center.y3);
  
	  double cx0 = center.x0;
	  double cx1 = center.x1;
	  double cx2 = center.x2;
	  double cx3 = center.x3;

 	  double y0 = center.y0;
	  double y1 = center.y1;
 	  double y2 = center.y2;
 	  double y3 = center.y3;


  int i, j, k, n=3;

  double EqSys[3][4], Cvar[3];

	EqSys[0][0]= 1;
	EqSys[0][1]= cx0;
	EqSys[0][2]= cx0*cx0;
	EqSys[0][3]= y0;
	
	EqSys[1][0]= 1;
	EqSys[1][1]= cx1;
	EqSys[1][2]= cx1*cx1;
	EqSys[1][3]= y1;
	
	EqSys[2][0]= 1;
	EqSys[2][1]= cx2;
	EqSys[2][2]= cx2*cx2;
	EqSys[2][3]= y2;
	
	
  for (i=0;i<4;i++)                    //Pivotisation
        for (k=i+1;k<n;k++)
            if (abs(EqSys[i][i])<abs(EqSys[k][i]))
                for (j=0;j<=n;j++)
                {
                    double temp=EqSys[i][j];
                    EqSys[i][j]=EqSys[k][j];
                    EqSys[k][j]=temp;
                }

cout<<"\n\nThe matrix we are solving:\n";
    for (i=0;i<n;i++)            //print the new matrix
    {
        for (j=0;j<=n;j++)
            cout<<EqSys[i][j]<<setw(16);
        cout<<"\n";
    }


 //loop to perform the gauss elimination
  for (i=0;i<n-1;i++)           
        for (k=i+1;k<n;k++)
            {
                double t=EqSys[k][i]/EqSys[i][i];
                for (j=0;j<=n;j++)
                    EqSys[k][j]=EqSys[k][j]-t*EqSys[i][j];  
            }

cout<<"\n\nThe matrix after gauss-elimination is:\n";
    for (i=0;i<n;i++)            //print the new matrix
    {
        for (j=0;j<=n;j++)
            cout<<EqSys[i][j]<<setw(16);
        cout<<"\n";
    }


    for (i=n-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        Cvar[i]=EqSys[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=i+1;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
		Cvar[i]=Cvar[i]-EqSys[i][j]*Cvar[j];
		Cvar[i]=Cvar[i]/EqSys[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    cout<<"\nThe values of the variables are:\n";
     cout<<"C0 = "<<Cvar[0]<<endl;
     cout<<"C1 = "<<Cvar[1]<<endl;
     cout<<"C2 = "<<Cvar[2]<<endl<<endl;


cout<<"Road Model Equation: Y = "<<Cvar[0]<<" + "<<Cvar[1]<<"*X + "<<Cvar[2]<<"*X2 "<<endl;




	  parameter.c0=Cvar[0];
	  parameter.c1=Cvar[1];
	  parameter.c2=Cvar[2];

 	  parameter.y0 = center.y0;
	  parameter.y1 = center.y1;
 	  parameter.y2 = center.y2;
 	  parameter.y3 = center.y3;
 	  parameter.y00 = center.y00;

	  parameter.x0 = center.x0;
	  parameter.x1 = center.x1;
 	  parameter.x2 = center.x2;
 	  parameter.x3 = center.x3;

	  parameter.x0l=center.x0l;
	  parameter.x1l=center.x1l;
	  parameter.x2l=center.x2l;
	  parameter.x3l=center.x3l;

	  parameter.x0r=center.x0r;
	  parameter.x1r=center.x1r;
	  parameter.x2r=center.x2r;
	  parameter.x3r=center.x3r;




    pub_.publish(parameter);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "roadmodel1");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
