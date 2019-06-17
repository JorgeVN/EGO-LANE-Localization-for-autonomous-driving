#include <ros/ros.h>
#include "egolane/datax.h"

#include <sstream>
#include<iostream>
#include<fstream>
#include<iomanip>
#include <math.h>
#include <string>
#include <cmath>

using namespace std;
egolane::datax point;
const double PI = 4.0 * atan( 1.0 );
int counter = 0;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<egolane::datax>("LeftDistance", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("anglepredicted", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const egolane::datax& input)
  {
    egolane::datax output;
    //.... do something with the input and generate the output...
	
	point = input;	
	
	ROS_INFO("I heard ANGLE: [%f]", point.x0);
	ROS_INFO("I heard ANGLE: [%f]", point.y00);
	ROS_INFO("I heard ROAD LEFT Y POINTS: [%f, %f, %f, %f]", point.y0, point.y1, point.y2, point.y3);
  	ROS_INFO("I heard ROAD LEFT X POINTS: [%f, %f, %f, %f]", point.x0l, point.x1l, point.x2l, point.x3l);
  
	  double alpha = point.x0;	  
	  double xt = 320;

	  double cx0 = point.x0l;
	  double cx1 = point.x1l;
	  double cx2 = point.x2l;
	  double cx3 = point.x3l;

 	  double y0 = point.y0;
	  double y1 = point.y1;
 	  double y2 = point.y2;
 	  double y3 = point.y3;

	  int y00 = point.y00;

	 


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



			   double a = Cvar[2];
			   double b = Cvar[1];

	  double xp[3];
	  double xp0[250];
	  double xp1[250];
	  
	  int z = 0;	

	ofstream myfile0 ("0RoadLeftPoints0.txt", std::ios::app);

	ofstream myfile1 ("1RoadLeftPoints1.txt", std::ios::app);

	ofstream myfileOK ("3RoadLeftPointsOK.txt", std::ios::app);

	if (myfile0.is_open() && myfile1.is_open() && myfileOK.is_open())
	{
	  
  
	  for( int dy4 = y00; dy4 < 480; dy4++ ) {
		
		  //double dy4=290;
			   
		   	 double c = Cvar[0]-dy4;
			  double xp[2];

			double discriminant, realPart, imaginaryPart;
   

		    discriminant = b*b - 4*a*c;
		    
		    if (discriminant > 0) {
			xp[0] = (-b + sqrt(discriminant)) / (2*a);
			xp[1] = (-b - sqrt(discriminant)) / (2*a);
			//cout << "Roots are real and different." << endl;
			//cout << "x1 = " << xp[0] << endl;
			//cout << "x2 = " << xp[1] << endl;
	
		    }
		    
		    else if (discriminant == 0) {
			//cout << "Roots are real and same." << endl;
			//xp[0] = (-b + sqrt(discriminant)) / (2*a);
			//xp[1] = (-b + sqrt(discriminant)) / (2*a);
			//cout << "x1 = x2 =" << xp[0] << endl;
		    }

		    else {
			xp[0] = -b/(2*a);
			xp[1] = -b/(2*a);
			imaginaryPart =sqrt(-discriminant)/(2*a);
			//cout << "Roots are complex and different."  << endl;
			//cout << "x1 = " << xp[0] << "+" << imaginaryPart << "i" << endl;
			//cout << "x2 = " << xp[1] << "-" << imaginaryPart << "i" << endl;
		    }
		
		//saving roots in txt
		double x40 = xp[0];
		xp0[z] = xp[0];
		myfile0 << x40 <<"\n";
		double x41 = xp[1];
		xp1[z] = xp[1];
		myfile1 << x41 <<"\n";
		

		z = z+1;
  	  	int zz=480-y00;
	if (z==zz){
		//myfileOK<<"This is for the "<<n<<" msg\n";

		int z0 = 3;
		int z1 = (z-1)*0.25;
		int z2 = (z-1)*0.5;
		int z3 = (z-1)*0.75;

		double dif00 = abs (point.x0l - xp0[z0]);
		double dif01 = abs (point.x1l - xp0[z1]);
		double dif02 = abs (point.x2l - xp0[z2]);
		double dif03 = abs (point.x3l - xp0[z3]);
		double dif0 = dif00 + dif01 + dif02 + dif03;


		double dif10 = abs (point.x0l - xp1[z0]);
		double dif11 = abs (point.x1l - xp1[z1]);
		double dif12 = abs (point.x2l - xp1[z2]);
		double dif13 = abs (point.x3l - xp1[z3]);
		double dif1 = dif10 + dif11 + dif12 + dif13;

		double x4;

		if ( dif0<=dif1){
	
			for (int y=0;y<z; y++){
				x4 = xp0[y];
				myfileOK << x4 <<"\n";
				if(y==z-130){
				
				cout<<endl<<x4<<endl;
				double da = sqrt((xt-x4)*(xt-x4));	
				double ca = da*cos(alpha*(2*PI/360));
				cout<<"Left Distance = "<<ca<<endl<<endl<<endl;

				output.x0 = ca;
				pub_.publish(output);	
	
				}
			}

		}if ( dif1<=dif0 ){
	
			for (int y=0;y<z; y++){
				x4 = xp1[y];
				myfileOK << x4 <<"\n";
				if(y==z-130){
				
				cout<<endl<<x4<<endl;
				double da = sqrt((xt-x4)*(xt-x4));	
				double ca = da*cos(alpha*(2*PI/360));
				cout<<"Left Distance = "<<ca<<endl<<endl<<endl;

				output.x0 = ca;
				pub_.publish(output);	
	
				}
			}
		
		}
	}
			
  	  }//for
	myfile0.close();
	myfile1.close();
	myfileOK.close();
	n=n+1;
	}else {cout << "Unable to open file";}

    
  }



private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "leftdistance2");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

 return 0;
}
