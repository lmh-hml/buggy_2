#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/Quaternion.h>



void getTwistFromTransform( const  tf::Transform& tf, geometry_msgs::Twist* twist, geometry_msgs::Quaternion* quat)
{
    if(twist!=NULL)
    {
        tf::Vector3 translation = tf.getOrigin();
        twist->linear.x = translation.getX();
        twist->linear.z = translation.getZ();
        twist->linear.y = translation.getY();
    //    ROS_INFO("X: %.3f, Y:%.3f, Z:%.3f",twist->linear.x,twist->linear.y,twist->linear.z);
    }

    if(quat!=NULL)
    {
        tf::Quaternion q = tf.getRotation();
        quat->w = q.getW();
        quat->z = q.getZ();
        quat->y = q.getY();
        quat->x = q.getX();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_node");
    ros::NodeHandle nh;
    tf::TransformListener listener;

    ros::Publisher tf_mto_pub = nh.advertise<geometry_msgs::Twist>("mto_twist", 100);
    ros::Publisher tf_otb_pub = nh.advertise<geometry_msgs::Twist>("otb_twist", 100);
    ros::Publisher tf_mtb_pub = nh.advertise<geometry_msgs::Twist>("mtb_twist", 100);

    ros::Publisher quat_mto_pub= nh.advertise<geometry_msgs::Quaternion>("mto_quat", 100);
    ros::Publisher quat_otb_pub= nh.advertise<geometry_msgs::Quaternion>("otb_quat", 100);
    ros::Publisher quat_mtb_pub= nh.advertise<geometry_msgs::Quaternion>("mtb_quat", 100);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("tf_vel", 100);

    std::string dest_frame, src_frame, base_frame;
    nh.param<std::string>("dest_frame",dest_frame,"/odom");
    nh.param<std::string>("src_frame",src_frame,"/map");
    nh.param<std::string>("base_frame",base_frame,"/base_link");


    tf::StampedTransform map_to_odom, odom_to_base, map_to_base;
    geometry_msgs::Twist twist;
    ros::Rate r(50);
    ros::Time now, last;
    tf::Vector3 old_origin;
    tf::Quaternion old_quat;

    while(ros::ok())
    {
        ros::spinOnce();
        now = ros::Time::now();
        double dt  = (now-last).toSec();

        try
        {
            //reverse src and dest becoz return tf is reversed
            listener.lookupTransform("odom", "base_link", ros::Time(0),odom_to_base);
            listener.lookupTransform("map", "odom", ros::Time(0), map_to_odom);
            listener.lookupTransform("map", "base_link", ros::Time(0),map_to_base);

            listener.lookupTwist("base_link", "map", ros::Time(0),ros::Duration(1),twist);

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist translation;
        geometry_msgs::Twist trans_stamped;
        geometry_msgs::Quaternion quaternion;

        ROS_INFO("OTB: ");
        getTwistFromTransform(odom_to_base, &translation, &quaternion);
        quat_otb_pub.publish(quaternion);
        tf_otb_pub.publish(translation);

        ROS_INFO("MTO: ");
        getTwistFromTransform(map_to_odom, &translation, &quaternion);
        quat_mto_pub.publish(quaternion);
        tf_mto_pub.publish(translation);

        ROS_INFO("MTB: ");
        getTwistFromTransform(map_to_base, &translation, &quaternion);
        quat_mtb_pub.publish(quaternion);
        tf_mtb_pub.publish(translation);

        vel_pub.publish(twist);
        last = now;
    }

    return 0;
}
