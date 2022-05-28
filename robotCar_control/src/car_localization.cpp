#include"ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include"geometry_msgs/PointStamped.h"
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/TwistStamped.h"
#include"tf2/LinearMath/Matrix3x3.h"

void doLocation(const gazebo_msgs::ModelStatesConstPtr &ptr)
{     ros::NodeHandle nh1;
      ros::Publisher pub1=nh1.advertise<geometry_msgs::PoseStamped>("car_center_pose",100);
      ros::Publisher pub2=nh1.advertise<geometry_msgs::TwistStamped>("car_velocity",100);
      ros::Publisher pub3=nh1.advertise<geometry_msgs::PoseStamped>("car_rearCenter_pose",100);
      geometry_msgs::PoseStamped car_center_pose;
      geometry_msgs::PoseStamped car_realCenter_pose;
      geometry_msgs::TwistStamped car_vel;
      int num =ptr->name.size();
      ROS_INFO("the num is: %d",num);
      for(int i=0;i<num;i++){
            if (ptr->name[i]=="robot_car")
            {    ROS_INFO("i  is: %d",i);
                  car_center_pose.header.frame_id="/world";
                  car_center_pose.header.stamp=ros::Time::now();
                  car_center_pose.pose.orientation=ptr->pose[i].orientation;
                   car_center_pose.pose.position=ptr->pose[i].position;
                  tf2::Quaternion  qa;
                  tf2::Matrix3x3 mat(tf2::Quaternion(car_center_pose.pose.orientation.x,
                  car_center_pose.pose.orientation.y,
                  car_center_pose.pose.orientation.z,
                  car_center_pose.pose.orientation.w));
                  double roll, pitch, yaw;
                  mat.getEulerZYX(roll,pitch,yaw);
                   car_realCenter_pose.header.frame_id="/wrold";
                   car_realCenter_pose.header.stamp=ros::Time::now();
                   double wheelbase=3;
                   car_realCenter_pose.pose.position.x=car_center_pose.pose.position.x-0.5*wheelbase*cos(yaw) ;
                  car_realCenter_pose.pose.position.y=car_center_pose.pose.position.y-0.5*wheelbase*sin(yaw);
                  car_realCenter_pose.pose.position.z=car_center_pose.pose.position.z;
                  car_realCenter_pose.pose.orientation=car_center_pose.pose.orientation;

                  car_vel.header.frame_id="/world";
                  car_vel.header.stamp=ros::Time::now();
                  car_vel.twist=ptr->twist[i];
                  ROS_INFO("FIND..............");
                  break;
            }
      }

            ros::Rate rate=10;
            while(ros::ok){
             pub1.publish(car_center_pose);
             pub2.publish(car_vel);
             pub3.publish(car_realCenter_pose);
            ros::spinOnce();
            rate.sleep();
            }


      
   
      //car_pose.header.frame_id="base_link";
     
      

}
int main(int argc, char * argv[])
{
      ros::init(argc,argv,"car_location");
      ros::NodeHandle nh;

      ros::Subscriber sub=nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,doLocation);

      ros::spin();


      return 0;
}