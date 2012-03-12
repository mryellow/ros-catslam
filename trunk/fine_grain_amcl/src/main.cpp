#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

double px,py,pz,rx,ry,rz,rw;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"fine_grain_amcl");
    ros::NodeHandle n;
    ros::Publisher raw_pub= n.advertise<geometry_msgs::PointStamped>("fine_grain_amcl",1);
    ros::Rate loop_rate(10);
    int count = 0;
    tf::TransformListener listener;
    while(ros::ok())
    {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        //ROS_INFO("Robot at %f,%f",transform.getOrigin().x(),transform.getOrigin().y() );
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/map";
        msg.pose.position.x = transform.getOrigin().x();
        msg.pose.position.y = transform.getOrigin().y();
        msg.pose.position.z = transform.getOrigin().z();
        msg.pose.orientation.x = transform.getRotation().x();
        msg.pose.orientation.y = transform.getRotation().y();
        msg.pose.orientation.z = transform.getRotation().z();
        msg.pose.orientation.w = transform.getRotation().w();
        raw_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
