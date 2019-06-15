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

	  double x0m = point.x0l;


  int i, j, k, n=4;

  double EqSys[4][5], Cvar[4];

	EqSys[0][0]= 1;
	EqSys[0][1]= cx0;
	EqSys[0][2]= cx0*cx0;
	EqSys[0][3]= cx0*cx0*cx0;
	EqSys[0][4]= y0;
	
	EqSys[1][0]= 1;
	EqSys[1][1]= cx1;
	EqSys[1][2]= cx1*cx1;
	EqSys[1][3]= cx1*cx1*cx1;
	EqSys[1][4]= y1;
	
	EqSys[2][0]= 1;
	EqSys[2][1]= cx2;
	EqSys[2][2]= cx2*cx2;
	EqSys[2][3]= cx2*cx2*cx2;
	EqSys[2][4]= y2;
	
	EqSys[3][0]= 1;
	EqSys[3][1]= cx3;
	EqSys[3][2]= cx3*cx3;
	EqSys[3][3]= cx3*cx3*cx3;
	EqSys[3][4]= y3;
	
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
     cout<<"C2 = "<<Cvar[2]<<endl;
     cout<<"C3 = "<<Cvar[3]<<endl<<endl;

			   double a = Cvar[3];
			   double b = Cvar[2];
			   double c = Cvar[1];
			   double dd = Cvar[0];

cout<<"Road Model Equation: Y = "<<Cvar[0]<<" + "<<Cvar[1]<<"*X + "<<Cvar[2]<<"*X2 + "<<Cvar[3]<<"*X3"<<endl;

  	  double xp[3];
	  double xp0[500];
	  double xp1[500];
	  double xp2[500];
	  int z = 0;

ofstream myfile0 ("0RoadLeftPoints0.txt", std::ios::app);

	ofstream myfile1 ("1RoadLeftPoints1.txt", std::ios::app);
	ofstream myfile2 ("2RoadLeftPoints2.txt", std::ios::app);
	ofstream myfileOK ("3RoadLeftPointsOK.txt", std::ios::app);

	if (myfile0.is_open() && myfile1.is_open() && myfile2.is_open() && myfileOK.is_open())
	{
	  
  
	  for( int dy4 = y00; dy4 < 480; dy4++ ) {
		
		  //double dy4=290;
			  double d = dd - dy4;
			   
			  double xp[3];

			  // Reduced equation: X^3 - 3pX - 2q = 0, where X = x-b/(3a)
		   double p = ( b * b - 3.0 * a * c ) / ( 9.0 * a * a );
		   double q = ( 9.0 * a * b * c - 27.0 * a * a * d - 2.0 * b * b * b ) / ( 54.0 * a * a * a );
		   double offset = b / ( 3.0 * a );

		   // Discriminant
		   double discriminant = p * p * p - q * q;


		   //cout << "\nRoots:\tY="<<dy4<<endl;

		   if ( discriminant > 0 )           // set X = 2 sqrt(p) cos(theta) and compare 4 cos^3(theta)-3 cos(theta) = cos(3 theta)
		   {
		      double theta = acos( q / ( p * sqrt( p ) ) );
		      double r = 2.0 * sqrt( p );
		      for ( int n = 0; n < 3; n++ ) {
			//cout << r * cos( ( theta + 2.0 * n * PI ) / 3.0 ) - offset << '\n';
			xp[n]=r * cos( ( theta + 2.0 * n * PI ) / 3.0 ) - offset;	
			}
		   }
		   else 
		   {
		      double gamma1 = cbrt( q + sqrt( -discriminant ) );
		      double gamma2 = cbrt( q - sqrt( -discriminant ) );

		      //cout << gamma1 + gamma2 - offset << '\n';
		      xp[0]=gamma1 + gamma2 - offset;

		      double re = -0.5 * ( gamma1 + gamma2 ) - offset;
		      double im = ( gamma1 - gamma2 ) * sqrt( 3.0 ) / 2.0;
		      if ( discriminant == 0.0 )                // Equal roots (hmmm, floating point ...)
		      {
			 //cout << re << '\n';
			  xp[1]= re;
			 //cout << re << '\n';
			  xp[2]= re;
		      }
		      else
		      {
			 //cout << re << " + " << im << " i\n";
			  xp[1]= re;
			 //cout << re << " - " << im << " i\n";
			  xp[2]= re;
		      }
		   }

		double x4;

		
		//saving roots in txt
		double x40 = xp[0];
		xp0[z] = xp[0];
		myfile0 << x40 <<"\n";
		double x41 = xp[1];
		xp1[z] = xp[1];
		myfile1 << x41 <<"\n";
		double x42 = xp[2];
		xp2[z] = xp[2];
		myfile2 << x42 <<"\n";

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


		double dif20 = abs (point.x0l - xp2[z0]);
		double dif21 = abs (point.x1l - xp2[z1]);
		double dif22 = abs (point.x2l - xp2[z2]);
		double dif23 = abs (point.x3l - xp2[z3]);
		double dif2 = dif20 + dif21 + dif22 + dif23;

		double x4;

		if ( dif0<=dif1 && dif0<=dif2){
	
			for (int y=0;y<z; y++){
				x4 = xp0[y];
				myfileOK << x4 <<"\n";
				if(y==z-1){
					
				cout<<endl<<x4<<endl;
				cout<<endl<<alpha<<endl;
				double da = sqrt((xt-x4)*(xt-x4));
				cout<<endl<<"da = "<<da<<endl;
				cout<<endl<<"xt = "<<xt<<endl;		
				double ca = da*cos(alpha*(2*PI/360));
				cout<<"Y = "<<z-1+y00<<endl;
				cout<<"Left Distance = "<<ca<<endl<<endl<<endl;

				output.x0 = ca;
				pub_.publish(output);	

				}
			}

		}else if ( dif1<=dif0 && dif1<=dif2){
	
			for (int y=0;y<z; y++){
				x4 = xp1[y];
				myfileOK << x4 <<"\n";
				if(y==z-1){
					
				cout<<endl<<x4<<endl;
				cout<<endl<<alpha<<endl;
				double da = sqrt((xt-x4)*(xt-x4));
				cout<<endl<<"da = "<<da<<endl;
				cout<<endl<<"xt = "<<xt<<endl;		
				double ca = da*cos(alpha*(2*PI/360));
				cout<<"Y = "<<z-1+y00<<endl;
				cout<<"Left Distance = "<<ca<<endl<<endl<<endl;

				output.x0 = ca;
				pub_.publish(output);	

				}
			}
		}else if ( dif2<=dif0 && dif2<=dif1){
			for (int y=0;y<z; y++){
				x4 = xp2[y];
				myfileOK << x4 <<"\n";
				if(y==z-1){
					
				cout<<endl<<"x4 = "<<x4<<endl;
				cout<<endl<<"alpha = "<<alpha<<endl;
				double da = sqrt((xt-x4)*(xt-x4));
				cout<<endl<<"da = "<<da<<endl;
				cout<<endl<<"xt = "<<xt<<endl;	
				double ca = da*cos(alpha*(2*PI/360));
				cout<<"Y = "<<z-1+y00<<endl;
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
	myfile2.close();
	myfileOK.close();

	counter=counter+1;
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
  ros::init(argc, argv, "leftdistance");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

 return 0;
}
