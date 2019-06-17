#include <ros/ros.h>
#include "egolane/datax.h"

#include <sstream>
#include<iostream>
#include<iomanip>
#include <math.h>
#include <string>
#include <cmath>

using namespace std;
egolane::datax point;
const double PI = 4.0 * atan( 1.0 );

class SubscribeAndPublish
{
public:
  
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<egolane::datax>("anglepredicted", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("PredictedPoint", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const egolane::datax& input)
  {
    egolane::datax angled;
	
	point = input;	
	
	ROS_INFO("I heard POINTS: [%f, %f]", point.xp, point.yp);
	ROS_INFO("I heard ROAD PARAMETERS: C1, C2 [%f, %f]", point.x2, point.x3);
  
	  double x4 = point.xp;
	  double y4 = point.yp;

	  double C1 = point.x2;
	  double C2 = point.x3;

		//Recta tangente	Angular Displacement
		//y-f(x4)=f'(x4)*(x-x4) ->  y = m*x - m*x4 + y4  ->  y = m*x + cn
		//m=f'(x4)			cn = - m*x4 + y4

		  double m = C1 + 2*C2*x4;
		  double cn = -m*x4 + y4;
		  cout<<endl<<"Tangent line: y = "<< m <<"*x + "<< cn <<endl<<endl;
		  
		  double xt = 320; //centro de la imagen
		  double yt = m*xt + cn;
		  cout<<"y for x=320 (center of the image) = "<< yt <<endl;  

		  double ds = sqrt((xt-x4)*(xt-x4));
		  double dc = sqrt((yt-y4)*(yt-y4));
		  cout<<"distance from x4 to the x center of the image = "<< dc <<endl;
		  cout<<"distance from y tangent line to y4 = "<< ds <<endl;
			

		 
		  //angle displacement to the rigth
		  //you need to turn left
		  if (-m<0){ 
			double angle = -atan(ds/dc) * 180 / PI; //grados
			cout<<endl<<"Angle displacement = "<< angle <<"º"<<endl<<endl;
			angled.x0=angle;

				  angled.y0 = point.y0;
				  angled.y1 = point.y1;
			 	  angled.y2 = point.y2;
			 	  angled.y3 = point.y3;
			 	  angled.y00 = point.y00;

				  angled.x0l=point.x0l;
				  angled.x1l=point.x1l;
				  angled.x2l=point.x2l;
				  angled.x3l=point.x3l;

				  angled.x0r=point.x0r;
				  angled.x1r=point.x1r;
				  angled.x2r=point.x2r;
				  angled.x3r=point.x3r;


			pub_.publish(angled);
		  }
		  //angle displacement to the left
		  //you need to turn right
		  else{
		  	double angle = atan(ds/dc) * 180 / PI; //grados
			cout<<endl<<"Angle displacement = "<< angle <<"º"<<endl<<endl;
			angled.x0=angle;

			 	  angled.y0 = point.y0;
				  angled.y1 = point.y1;
			 	  angled.y2 = point.y2;
			 	  angled.y3 = point.y3;
			 	  angled.y00 = point.y00;

				  angled.x0l=point.x0l;
				  angled.x1l=point.x1l;
				  angled.x2l=point.x2l;
				  angled.x3l=point.x3l;

				  angled.x0r=point.x0r;
				  angled.x1r=point.x1r;
				  angled.x2r=point.x2r;
				  angled.x3r=point.x3r;

			pub_.publish(angled);
		  }
		  


  

	 /* angled.x0=angle;
	  parameter.x1=Cvar[1];
	  parameter.x2=Cvar[2];
	  parameter.x3=Cvar[3];


    pub_.publish(angled);*/
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "getangle");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject; // SAPObject.x4

  ros::spin();

  return 0;
}
