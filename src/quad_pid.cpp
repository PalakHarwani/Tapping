//#include "ark_llp/go2goal.h"
//#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sstream>
#include <math.h>
#define Default 0.5
nav_msgs::Odometry obspose;
//nav_msgs::Odometry quadpose;
geometry_msgs::PoseStamped pose;
nav_msgs::Odometry pos_feed;
float current_x;
float current_y;
float current_z;
void obsCallback(const nav_msgs::Odometry::ConstPtr& msg);
float GetErrorLin(nav_msgs::Odometry obspose,float current_x,float current_y);

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    Go2Goal random;
    double x, y, z, theta;
    float func;
    x = std::atof(argv[1]);
    y = std::atof(argv[2]);
    z = std::atof(argv[3]);
    theta = std::atof(argv[4]);
    func = std::atof(argv[5]);
    //double x=1,y=2,z=3,theta=0;
    //double x1=0.1,y1=0.2,z1=3,theta1=0;
    if(func==1)
    	random.set_dest(x, y, z, theta);
    else
    	random.false_dest(x, y, z, theta);
    return 0;

}*/

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    pos_feed= *odom_data;
    current_x = pos_feed.pose.pose.position.x;
   current_y= pos_feed.pose.pose.position.y;
   current_z= pos_feed.pose.pose.position.z;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"quad_pid");
  double x, y, z, theta,kp;
  float ErrorLin,ErrorQuad,ErrorObs ;
  float obs_theta;
  float set_theta;
  x = std::atof(argv[1]);
  y = std::atof(argv[2]);
  kp=std::atof(argv[3]);//0.8
  theta = 0;
  ros::NodeHandle nh;
  //ros::Subscriber quadpose_sub = nh.subscribe("/ground_truth/state", 100, quadCallback); // subscriber to get MAV position
  ros::Subscriber quadpose_sub = nh.subscribe("mavros/local_position/odom", 10, feedbackfn);
  ros::Subscriber obspose_sub = nh.subscribe("/robot3/odom", 100, obsCallback); // subscriber to get ground bot position
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
             ("mavros/setpoint_position/local", 10);
    ros::Rate rate(20.0);

    

      

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 0.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

     while(ros::ok())
  {

     if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

       
    obs_theta= atan2(((obspose.pose.pose.position.y)-(current_y)),((obspose.pose.pose.position.x)- (current_x)));
    set_theta= atan2((y-(current_y)),(x-(current_x)));

    ErrorLin=GetErrorLin(obspose,current_x,current_y);
    ErrorQuad = sqrt(pow((y -(current_y)),2) + pow((x - (current_x)),2));
    ErrorObs = sqrt(pow((y - (obspose.pose.pose.position.y)),2) + pow((x - (obspose.pose.pose.position.x)),2));
    if(ErrorLin < 3.5 && (ErrorObs<ErrorQuad))
    {
       // ROS_INFO("%f %f %f \n",current_x,current_y,current_z);
        //ROS_INFO("%f", ErrorQuad);
        z=kp*ErrorLin; 
        if((set_theta) <= (obs_theta))
         {
          ROS_INFO("%f -",ErrorLin);
          set_theta-=z;
         }
        else if((set_theta) > (obs_theta))
         {
          ROS_INFO("%f +",ErrorLin);
          set_theta+=z;
         }
                // dest.false_dest(((current_y)*(-1)+2*cos(set_theta)),(current_x+2*sin(set_theta)),Default,0);
    pose.pose.position.x = ((current_x)+0.3*cos(set_theta));
    pose.pose.position.y = ((current_y)+0.3*sin(set_theta));
    pose.pose.position.z = Default;
    local_pos_pub.publish(pose);
    }

    else
    {
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = Default;
    local_pos_pub.publish(pose);
    }
    ros::spinOnce();
    rate.sleep();
  }
}

void obsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  obspose.pose.pose.position.x = msg->pose.pose.position.x;
  obspose.pose.pose.position.y = msg->pose.pose.position.y;
  obspose.pose.pose.position.z = msg->pose.pose.position.z;
  return;
}
/*void quadCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_x = msg->pose.pose.position.x;
 current_y = msg->pose.pose.position.y;
  quadpose.pose.pose.position.z = msg->pose.pose.position.z;
  return;
}*/

float GetErrorLin(nav_msgs::Odometry obspose,float current_x,float current_y)
{
  float El;
  El = sqrt(pow((current_x - obspose.pose.pose.position.x),2) + pow((current_y - obspose.pose.pose.position.y),2));
  return(El);
}
