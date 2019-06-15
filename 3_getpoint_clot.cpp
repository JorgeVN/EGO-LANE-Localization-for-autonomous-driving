
#include "ros/ros.h"
#include "egolane/datax.h"

#include <sstream>
#include<iostream>
#include<fstream>
#include<iomanip>
//#include <math.h>
#include <string>
#include <cmath>

using namespace std;


egolane::datax parameter;
const double PI = 4.0 * atan( 1.0 );
int n = 0;


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<egolane::datax>("PredictedPoint", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("parameters", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const egolane::datax& input)
  {
    egolane::datax pointx;
    //.... do something with the input and generate the output...
	
	parameter = input;	
	
	ROS_INFO("I heard: [%f, %f, %f, %f]", parameter.c0, parameter.c1, parameter.c2, parameter.c3);

	//double roadPoints[];

			   double x0l = parameter.x0l;
			   double x0r = parameter.x0r;
		
			   double x0m = (x0l + x0r)/2;

			   int y00 = parameter.y00;
			   double y0 = parameter.y0;
			   double y1 = parameter.y1;
			   double y2 = parameter.y2;
			   double y3 = parameter.y3;

			   double x0 = parameter.x0;
			   double x1 = parameter.x1;
			   double x2 = parameter.x2;
			   double x3 = parameter.x3;


	  double xp[3];
	  double xp0[500];
	  double xp1[500];
	  double xp2[500];
	  int z = 0;

	/*ofstream myfile0 ("0RoadCenterPoints0.txt", std::ios::app);

	ofstream myfile1 ("1RoadCenterPoints1.txt", std::ios::app);
	ofstream myfile2 ("2RoadCenterPoints2.txt", std::ios::app);
	ofstream myfileOK ("3RoadCenterPointsOK.txt", std::ios::app);

	if (myfile0.is_open() && myfile1.is_open() && myfile2.is_open() && myfileOK.is_open())
	{
	  myfile0<<"This is for the "<<n<<" msg\n";
	  myfile1<<"This is for the "<<n<<" msg\n";
	  myfile2<<"This is for the "<<n<<" msg\n";*/
	 

  
	  for( int dy4 = y00; dy4 < 480; dy4++ ) {
		
			  //double dy4=290;	   
		  double a = parameter.c3;
		  double b = parameter.c2;
		  double c = parameter.c1;
		  double d = parameter.c0-dy4;

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
			xp[1]=re;
			xp[2]=re; //para quedarme solo con un re
			 //cout << re << '\n';
			 //cout << re << '\n';
		      }
		      else
		      {
			xp[1]=re;
			xp[2]=re; //para quedarme solo con un re
			 //cout << re << " + " << im << " i\n";
			 //cout << re << " - " << im << " i\n";
		      }
		   }
		

		//saving roots in txt
		double x40 = xp[0];
		xp0[z] = xp[0];
		//myfile0 << x40 <<"\n";
		double x41 = xp[1];
		xp1[z] = xp[1];
		//myfile1 << x41 <<"\n";
		double x42 = xp[2];
		xp2[z] = xp[2];
		//myfile2 << x42 <<"\n";*/

		z = z+1;
  	  	int zz=480-y00;
		cout << "FUERA"<<endl;
	if (z==zz){
		//myfileOK<<"This is for the "<<n<<" msg\n";
		cout << "DENTRO"<<endl;
		int z0 = 3;
		int z1 = (z-1)*0.15;
		int z2 = (z-1)*0.40;
		int z3 = (z-1)*0.80;

		double dif00 = abs (parameter.x0 - xp0[z0]);
		double dif01 = abs (parameter.x1 - xp0[z1]);
		double dif02 = abs (parameter.x2 - xp0[z2]);
		double dif03 = abs (parameter.x3 - xp0[z3]);
		double dif0 = dif00 + dif01 + dif02 + dif03;


		double dif10 = abs (parameter.x0 - xp1[z0]);
		double dif11 = abs (parameter.x1 - xp1[z1]);
		double dif12 = abs (parameter.x2 - xp1[z2]);
		double dif13 = abs (parameter.x3 - xp1[z3]);
		double dif1 = dif10 + dif11 + dif12 + dif13;


		double dif20 = abs (parameter.x0 - xp2[z0]);
		double dif21 = abs (parameter.x1 - xp2[z1]);
		double dif22 = abs (parameter.x2 - xp2[z2]);
		double dif23 = abs (parameter.x3 - xp2[z3]);
		double dif2 = dif20 + dif21 + dif22 + dif23;

		double x4;

		if ( dif0<=dif1 && dif0<=dif2){
	
			for (int y=0;y<z; y++){
				x4 = xp0[y];
				//myfileOK << x4 <<"\n";
				if(y==z-130){
				
		


					cout<<"Point predicted for Y="<<y+y00<<" -->  X="<<x4<<endl<<endl;				
				
					pointx.xp=x4;
					pointx.yp=y+y00;
					pointx.x1=c;
					pointx.x2=b;
					pointx.x3=a;

				 	pointx.y0 = parameter.y0;
					pointx.y1 = parameter.y1;
				 	pointx.y2 = parameter.y2;
				 	pointx.y3 = parameter.y3;
				 	pointx.y00 = parameter.y00;

					pointx.x0l=parameter.x0l;
					pointx.x1l=parameter.x1l;
					pointx.x2l=parameter.x2l;
					pointx.x3l=parameter.x3l;

					pointx.x0r=parameter.x0r;
					pointx.x1r=parameter.x1r;
					pointx.x2r=parameter.x2r;
					pointx.x3r=parameter.x3r;


					pub_.publish(pointx);	


				}
			}

		}else if ( dif1<=dif0 && dif1<=dif2){
	
			for (int y=0;y<z; y++){
				x4 = xp1[y];
				//myfileOK << x4 <<"\n";
				if(y==z-130){
				
		


					cout<<"Point predicted for Y="<<y+y00<<" -->  X="<<x4<<endl<<endl;				
				
					pointx.xp=x4;
					pointx.yp=y+y00;
					pointx.x1=c;
					pointx.x2=b;
					pointx.x3=a;

				 	pointx.y0 = parameter.y0;
					pointx.y1 = parameter.y1;
				 	pointx.y2 = parameter.y2;
				 	pointx.y3 = parameter.y3;
				 	pointx.y00 = parameter.y00;

					pointx.x0l=parameter.x0l;
					pointx.x1l=parameter.x1l;
					pointx.x2l=parameter.x2l;
					pointx.x3l=parameter.x3l;

					pointx.x0r=parameter.x0r;
					pointx.x1r=parameter.x1r;
					pointx.x2r=parameter.x2r;
					pointx.x3r=parameter.x3r;


					pub_.publish(pointx);	


				}
			}
		}else if ( dif2<=dif0 && dif2<=dif1){
			for (int y=0;y<z; y++){
				x4 = xp2[y];
				//myfileOK << x4 <<"\n";
				if(y==z-130){
				
		


					cout<<"Point predicted for Y="<<y+y00<<" -->  X="<<x4<<endl<<endl;				
				
					pointx.xp=x4;
					pointx.yp=y+y00;
					pointx.x1=c;
					pointx.x2=b;
					pointx.x3=a;

				 	pointx.y0 = parameter.y0;
					pointx.y1 = parameter.y1;
				 	pointx.y2 = parameter.y2;
				 	pointx.y3 = parameter.y3;
				 	pointx.y00 = parameter.y00;

					pointx.x0l=parameter.x0l;
					pointx.x1l=parameter.x1l;
					pointx.x2l=parameter.x2l;
					pointx.x3l=parameter.x3l;

					pointx.x0r=parameter.x0r;
					pointx.x1r=parameter.x1r;
					pointx.x2r=parameter.x2r;
					pointx.x3r=parameter.x3r;


					pub_.publish(pointx);	
					


				}
			}
		}else{cout<<"ELSEEEEEE"<<endl;}
	}
}//for

	/*myfile0.close();
	myfile1.close();
	myfile2.close();
	myfileOK.close();*/
	//n=n+1;
	//}else {cout << "Unable to open file";}

    
  
}

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "verify");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}



