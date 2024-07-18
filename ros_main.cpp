#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <sensor_msgs/Image.h>

#include <geometry_msgs/Twist.h>
#include <my_ros_test/franka_tle493d_syn.h>
#include <geometry_msgs/TwistStamped.h>
#include <tle493dw2b6_nano_processor/MagneticFieldVector.h>

// using namespace sensor_msgs;
using namespace message_filters;
using namespace my_ros_test;
using namespace geometry_msgs;
using namespace tle493dw2b6_nano_processor;

ros::Publisher pub;

void callback(const MagneticFieldVectorConstPtr &magnetic_msg,
              const TwistStampedConstPtr &eevel_msg)
{
    franka_tle493d_syn msg;
    msg.header.stamp = ros::Time::now();
    msg.magnetic_x = magnetic_msg->magnetic_x;
    msg.magnetic_y = magnetic_msg->magnetic_y;
    msg.magnetic_z = magnetic_msg->magnetic_z;
    msg.twist.linear.x = eevel_msg->twist.linear.x;
    msg.twist.linear.y = eevel_msg->twist.linear.y;
    msg.twist.linear.y = eevel_msg->twist.linear.y;
    msg.twist.angular.x = eevel_msg->twist.angular.x;
    msg.twist.angular.y = eevel_msg->twist.angular.y;
    msg.twist.angular.z = eevel_msg->twist.angular.z;
    pub.publish(msg);
}   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_sampling");

    ros::NodeHandle nh;
    
    // Dyx: we should wait for both topic have first received message 
    MagneticFieldVector::ConstPtr m = ros::topic::waitForMessage<MagneticFieldVector>("/MagneticSensor");
    ROS_INFO("Mag!");
    TwistStamped::ConstPtr t = ros::topic::waitForMessage<TwistStamped>("/Endpoint_vel");
    ROS_INFO("Start!");

    pub = nh.advertise<franka_tle493d_syn>("/sensor_data", 10);
    message_filters::Subscriber<MagneticFieldVector> magnetic_sub(
        nh, "/MagneticSensor", 1);
    message_filters::Subscriber<TwistStamped> eevel_sub(
        nh, "/Endpoint_vel", 1);

    typedef sync_policies::ApproximateTime<MagneticFieldVector, TwistStamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), magnetic_sub, eevel_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}