#include "ros/ros.h"
#include "string"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Mavlink.h"
#include "std_msgs/Int16.h"
#include "mavros_msgs/AttitudeTarget.h"

#define KILL 2
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
mavros_msgs::AttitudeTarget T;


std_msgs::Bool take_single;
std_msgs::Int16 sys_kill;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void kill_cb(const std_msgs::Int16::ConstPtr& msg)
{
    sys_kill = *msg;
}


void cmd_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    T.type_mask = T.IGNORE_PITCH_RATE | \
    T.IGNORE_ROLL_RATE |T.IGNORE_YAW_RATE ;

    T.thrust = msg->data[0];
    T.orientation.w = msg -> data[1];
    T.orientation.x = msg -> data[2];
    T.orientation.y = msg -> data[3];
    T.orientation.z = msg -> data[4];
}

int main(int argv,char** argc)
{
    ros::init(argv,argc,"MAV3");
    ros::NodeHandle nh;
    int UAV_ID;
    ros::Subscriber kill = nh.subscribe<std_msgs::Int16>
        ("/system/trajectory",10,kill_cb);
   
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    ros::Publisher T_pub = nh.advertise<mavros_msgs::AttitudeTarget >
            ("mavros/setpoint_raw/attitude", 10);

    ros::Subscriber cmd_sub = nh.subscribe<std_msgs::Float64MultiArray>
        ("cmd",10,cmd_cb);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

    //wait all MAV node are execute
    ros::topic::waitForMessage<std_msgs::Bool>("/arm");

    ros::Rate rate(100);    
    while(ros::ok() && !current_state.connected){

        ros::spinOnce();
        rate.sleep();
    }
 

    ros::param::get("UAV_ID", UAV_ID);
    ROS_INFO("Wait for setting origin and home position...");
    std::string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic);
    ROS_INFO("setting origin success!!");

 
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    ros::topic::waitForMessage<std_msgs::Bool>("/MAV/takeoff");
    
    //Take_off

    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 0.5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff se/uav0/nd ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }



    sleep(5);
    ros::topic::waitForMessage<std_msgs::Float64MultiArray>("cmd");
    while(ros::ok() && sys_kill.data != KILL)
    {
        T_pub.publish(T);

        ros::spinOnce();
        rate.sleep();

    }

    ROS_WARN("kill!");
    offb_set_mode.request.custom_mode = "LAND";
    set_mode_client.call(offb_set_mode);
    arm_cmd.request.value = false;
    arming_client.call(arm_cmd);
    sleep(3);
    return 0;
}