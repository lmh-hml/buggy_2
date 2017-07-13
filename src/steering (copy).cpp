#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>

ros::Publisher jt_pub;
ros::Subscriber sub;
geometry_msgs::Twist cmd_vel;

int linearX,linearY,angularZ;

void JoyCallback( const sensor_msgs::Joy::ConstPtr& js)
{
	cmd_vel.linear.x  = (js->buttons[0]==0) ? js->axes[1] : 0.0; //to/fro
	cmd_vel.linear.y  = (1+js->axes[3]) / 2; //+/-
	cmd_vel.angular.z = js->axes[2]; //rotate left/right 

	ROS_INFO("cmd_vel: %.3f, %.3f, %.3f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
	
	jt_pub.publish( cmd_vel );
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"steering");
    ros::NodeHandle nh;
    
    nh.param("linearX",linearX,0);
    nh.param("linearY",linearY,3);
    nh.param("linearZ",linearZ,2);
    
    
    jt_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    sub=nh.subscribe<sensor_msgs::Joy>("joy",10, JoyCallback);
    
	ros::spin();
	
	return 0;
}
