	
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

	
	parameter = input;	
	
	ROS_INFO("I heard: [%f, %f, %f]", parameter.x0, parameter.x1, parameter.x2);



			  
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
	  double xp0[250];
	  double xp1[250];
	  
	  int z = 0;


	ofstream myfile0 ("0RoadCenterPoints0.txt", std::ios::app);

	ofstream myfile1 ("1RoadCenterPoints1.txt", std::ios::app);

	ofstream myfileOK ("3RoadCenterPointsOK.txt", std::ios::app);

	if (myfile0.is_open() && myfile1.is_open() && myfileOK.is_open())
	{
	 
  
	  for( int dy4 = y00; dy4 < 480; dy4++ ) {
		
		
			   double a = parameter.c2;
			   double b = parameter.c1;
			   double c = parameter.c0-dy4;
			   
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
			xp[0] = (-b + sqrt(discriminant)) / (2*a);
			xp[1] = (-b + sqrt(discriminant)) / (2*a);
			//cout << "x1 = x2 =" << x1 << endl;
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

		double x4;

		if ( dif0<=dif1){
	
			for (int y=0;y<z; y++){
				x4 = xp0[y];
				myfileOK << x4 <<"\n";
				if(y==z-130){
				
		


					cout<<"Point predicted for Y=350 -->  X="<<x4<<endl<<endl;				
				
					pointx.xp=x4;
					pointx.yp=dy4;
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

		}else if ( dif1<=dif0 ){
	
			for (int y=0;y<z; y++){
				x4 = xp1[y];
				myfileOK << x4 <<"\n";
				if(y==z-130){
				
		


					cout<<"Point predicted for Y=350 -->  X="<<x4<<endl<<endl;				
				
					pointx.xp=x4;
					pointx.yp=dy4;
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
  ros::init(argc, argv, "verify");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
