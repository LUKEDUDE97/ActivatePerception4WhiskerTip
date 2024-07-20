#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <whisker_customed_msg/MagneticFieldVector.h>
#include <whisker_customed_msg/EESensorState.h>

using namespace message_filters;
using namespace geometry_msgs;
using namespace whisker_customed_msg;

// Define a struct to store the state of the end effector and sensor data
struct EE_Sensor_State
{
public:
    ros::Time current_time;
    float deflection_moment; 
    float xpos_ee, ypos_ee, zrot_ee;
};

// Instantiate the state object
EE_Sensor_State state;

// EE & Sensor State Publisher
ros::Publisher pub;

// Callback function to handle synchronized messages
void callback(const MagneticFieldVectorConstPtr &sensor_msg,
              const PoseStampedConstPtr &frankaEE_msg)
{
    state.current_time = ros::Time::now();
    state.deflection_moment = sensor_msg->magnetic_y;
    state.xpos_ee = frankaEE_msg->pose.position.x;
    state.ypos_ee = frankaEE_msg->pose.position.y;
    state.zrot_ee = frankaEE_msg->pose.orientation.z;

    // Log the received data
    ROS_INFO("Time: %f, Deflection: %f, Position: (%f, %f), Orientation: %f",
             state.current_time.toSec(), state.deflection_moment,
             state.xpos_ee, state.ypos_ee, state.zrot_ee);

    // Publish the combined state
    EESensorState msg;
    msg.header.stamp = state.current_time;
    msg.magnetic_y = sensor_msg->magnetic_y;
    msg.pose_ee.position.x = frankaEE_msg->pose.position.x;
    msg.pose_ee.position.y = frankaEE_msg->pose.position.y;
    msg.pose_ee.orientation.z = frankaEE_msg->pose.orientation.z;
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Initialize the Master ROS node
    ros::init(argc, argv, "Master_node");
    ros::NodeHandle nh;

    // Wait for the first message from both topics
    MagneticFieldVector::ConstPtr s = ros::topic::waitForMessage<MagneticFieldVector>("/Sensor_state");
    PoseStamped::ConstPtr f = ros::topic::waitForMessage<PoseStamped>("/FrankaEE_state");

    // Initialize the publisher
    pub = nh.advertise<EESensorState>("/EE_Sensor_state", 10);

    // Subscribe to the /Sensor_state and /FrankaEE_state topics
    message_filters::Subscriber<MagneticFieldVector> sensor_sub(nh, "/Sensor_state", 1);
    message_filters::Subscriber<PoseStamped> frankaEE_sub(nh, "/FrankaEE_state", 1);

    // Define the synchronization policy and synchronizer
    typedef sync_policies::ApproximateTime<MagneticFieldVector, PoseStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sensor_sub, frankaEE_sub);

    // Register the callback with the synchronizer
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Spin to process incoming messages
    ros::spin();

    return 0;
}
