#ifndef __TF_NODE_H__
#define __TF_NODE_H__
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <rosbag/bag.h>
#include <signal.h>

void getPoseFromTransform( const  tf::Transform& tf, geometry_msgs::PoseStamped* pose, const ros::Time& now, const char* frame )
{
    static uint id = 0;

    if(pose!=NULL)
    {
        id++;
        pose->header.seq = id;
        pose->header.stamp = now.now();
        pose->header.frame_id = frame;
        tf::Vector3 translation = tf.getOrigin();
        pose->pose.position.x = translation.getX();
        pose->pose.position.z = translation.getZ();
        pose->pose.position.y = translation.getY();
    //    ROS_INFO("X: %.3f, Y:%.3f, Z:%.3f",twist->linear.x,twist->linear.y,twist->linear.z);

        tf::Quaternion q = tf.getRotation();
        pose->pose.orientation.w = q.getW();
        pose->pose.orientation.x = q.getX();
        pose->pose.orientation.z = q.getY();
        pose->pose.orientation.z = q.getZ();
    }
}

class TF_Node
{
public:

    TF_Node(ros::NodeHandle* nh, ros::NodeHandle* nh_priv);

    ~TF_Node()
    {
        bag.close();
    };

    void twistCallback( const geometry_msgs::Twist::ConstPtr& twist);

    void loop();

    void writeBag();

    void close()
    {
        writeBag();
        printf("MTB_PATH SIZE: %lu, MTB_POSE ARRAY: %lu\n", mtb_path.poses.size(), mtb_poseArray.poses.size());
        printf("OTB_PATH SIZE: %lu, MTB_POSE ARRAY: %lu\n", otb_path.poses.size(), otb_poseArray.poses.size());
    }


private:
    //objects used to lookup tf and store poses to be published
    tf::StampedTransform       map_to_odom, odom_to_base, map_to_base;
    std_msgs::Header           mto_header,mtb_header,otb_header;
    geometry_msgs::PoseStamped mto_pose,otb_pose,mtb_pose;

    //Record of poses. Relevant only when dest frame is base_link (aka the position of the buggy relative to src frame)
    geometry_msgs::PoseArray   mtb_poseArray, otb_poseArray;
    nav_msgs::Path             mtb_path, otb_path;
    geometry_msgs::Twist       m_twist;

    ros::Publisher pose_mto_pub;
    ros::Publisher pose_otb_pub;
    ros::Publisher pose_mtb_pub;
    ros::Publisher path_mtb_pub;
    ros::Publisher path_otb_pub;
    ros::Publisher poseArray_mtb_pub;
    ros::Publisher poseArray_otb_pub;

    ros::Subscriber twist_listener;
    tf::TransformListener listener;

    rosbag::Bag bag;
    bool bag_on;
    std::string bag_dir;
    std::string bag_name;

    //params:
    std::string cmd_vel_topic;
    double period;                       //collect every *period* seconds

    ros::Time now, last;
    int loop_count;
};


TF_Node::TF_Node(ros::NodeHandle* nh, ros::NodeHandle* nh_priv)
{
    nh->param<bool>("bag_on",bag_on,true);
    nh->param<std::string>("cmd_vel",cmd_vel_topic,"/cmd_vel_out");

    nh_priv->param<std::string>("bag_dir" ,bag_dir,"/home/buggy/catkin_ws/src/buggy_core/bag/");
    nh_priv->param<std::string>("bag_name",bag_name,"tf_node.bag");
    nh_priv->param<double>("period",period,1.0);
    nh_priv->getParam("period",period);   //rosrun buggy_control tf_node _period:=2.0 to change period

    printf("%.3f s per lookup\n",period);

    pose_mto_pub      = nh->advertise<geometry_msgs::PoseStamped>("mto_pose", 100);
    pose_otb_pub      = nh->advertise<geometry_msgs::PoseStamped>("otb_pose", 100);
    pose_mtb_pub      = nh->advertise<geometry_msgs::PoseStamped>("mtb_pose", 100);
    path_mtb_pub      = nh->advertise<nav_msgs::Path>("mtb_path", 100);
    path_otb_pub      = nh->advertise<nav_msgs::Path>("otb_path", 100);
    poseArray_mtb_pub = nh->advertise<geometry_msgs::PoseArray>("mtb_pathArray", 100);
    poseArray_otb_pub = nh->advertise<geometry_msgs::PoseArray>("otb_pathArray", 100);
    twist_listener    = nh->subscribe<geometry_msgs::Twist>(cmd_vel_topic,10,&TF_Node::twistCallback,this);

    mto_header.frame_id = "map";
    mtb_header.frame_id = "map";
    otb_header.frame_id = "odom";
    now = last = ros::Time::now();


    try
    {
        bag.open( bag_dir+bag_name, rosbag::bagmode::Write);
    }
    catch(rosbag::BagIOException ex)
    {
        ROS_ERROR("%s",ex.what());
        bag_on = false;
    }
}

void TF_Node::twistCallback( const geometry_msgs::Twist::ConstPtr& twist)
{
    m_twist = *twist;
}

void TF_Node::loop()
{
    now = ros::Time::now();
    double time_elapsed  = (now-last).toSec();

    try
    {
        //reverse src and dest becoz returned tf is reversed
        listener.lookupTransform("odom", "base_link", ros::Time(0),odom_to_base);
        listener.lookupTransform("map", "odom", ros::Time(0), map_to_odom);
        listener.lookupTransform("map", "base_link", ros::Time(0),map_to_base);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep(); //retry after 1s
        return; //skip the rest
    }

    //increment if code manages to reach here
    //get individual pose and publish it as much as possible
        loop_count++;
        //ROS_INFO("OTB: ");
        getPoseFromTransform(odom_to_base, &otb_pose, now, "odom");
        pose_otb_pub.publish(otb_pose);

        //ROS_INFO("MTO: ");
        getPoseFromTransform(map_to_odom, &mto_pose, now, "map");
        pose_mto_pub.publish(mto_pose);

        //ROS_INFO("MTB: ");
        getPoseFromTransform(map_to_base, &mtb_pose, now, "map");
        pose_mtb_pub.publish(mtb_pose);

        //poses shall only be collected and published during each period
        if( time_elapsed >= period )
        {
            mtb_path.poses.push_back(mtb_pose);
            mtb_path.header = mtb_header;

            mtb_poseArray.poses.push_back(mtb_pose.pose);
            mtb_poseArray.header = mtb_header;

            otb_path.poses.push_back(otb_pose);
            otb_path.header = otb_header;

            otb_poseArray.poses.push_back(otb_pose.pose);
            otb_poseArray.header = otb_header;

            //PUBLISH EVERY PERIOD
            path_mtb_pub.publish(mtb_path);
            poseArray_mtb_pub.publish(mtb_poseArray);

            path_otb_pub.publish(otb_path);
            poseArray_otb_pub.publish(otb_poseArray);

            //bag
            if(bag_on)
            {
                bag.write("bag/mtb_pose",    now,mtb_pose);
                bag.write("bag/otb_pose",    now,otb_pose);
                bag.write("bag/mto_pose",    now,mto_pose);
                bag.write("bag/cmd_vel_out", now,m_twist);
            }

            last = now;
        }
}

void TF_Node::writeBag()
{
    ROS_INFO("WRITING BAG FILE...");
    ros::Time time = ros::Time::now();
    bag.write("mtb_path",time,mtb_path);
    bag.write("otb_path",time,otb_path);

    bag.write("mtb_poseArray",time,mtb_poseArray);
    bag.write("otb_poseArray",time,otb_poseArray);
}





#endif
