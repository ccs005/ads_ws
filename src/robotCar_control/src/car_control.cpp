#include"ros/ros.h"
#include"std_msgs/Float64.h"
#include"geometry_msgs/Twist.h"
#include"math.h"

std_msgs::Float64 rrSpd, rlSpd,frStrPos,flStrPos;
ros::Time t1;
ros::Time t2;
ros::Duration d(0.2);

void doConmmand(const geometry_msgs::Twist::ConstPtr &p ){
      t1 =ros::Time::now();
      ROS_INFO("the command recived time:%.2f",t1.toSec());
      float wheel_base=3;
      float wheel_track=1.8;
      float wheel_radius =0.38;
      float max_inside_angle =0.6;
      float steering_radius_max_angle =wheel_base /tan(max_inside_angle);
      float angle_limit = atan(wheel_base/(steering_radius_max_angle+0.5*wheel_track));
     
      if (p->angular.z==0){
            
            flStrPos.data=0;
            frStrPos.data=0;
            rrSpd.data=p->linear.x/wheel_radius;
            rlSpd.data=p->linear.x/wheel_radius;
            return;
            }

      float steering_angle=p->angular.z;
      steering_angle<-1*angle_limit?-1*angle_limit:steering_angle;
      steering_angle>angle_limit?angle_limit:steering_angle;

      float R0=wheel_base/tan(steering_angle);
      float R1=R0-0.5*wheel_track;
      float R2=R0+0.5*wheel_track;

      flStrPos.data=atan(wheel_base/R1);
      frStrPos.data=atan(wheel_base/R2);

      rrSpd.data=p->linear.x *R1/R0/wheel_radius;
      rlSpd.data=p->linear.x*R2/R0/wheel_radius;

      ROS_INFO("control starts");
}

int main(int argc, char *argv[])
{
      ros::init(argc,argv,"pub");
      ros::NodeHandle nh;

      ros::Publisher  pub1=nh.advertise<std_msgs::Float64>("robot_car/rear_right_velocity_controller/command" ,100);
      ros::Publisher  pub2=nh.advertise<std_msgs::Float64>("robot_car/rear_left_velocity_controller/command" ,100);
      ros::Publisher  pub3=nh.advertise<std_msgs::Float64>("/robot_car/front_left_steering_position_controller/command" ,100);
      ros::Publisher  pub4=nh.advertise<std_msgs::Float64>("/robot_car/front_right_steering_position_controller/command" ,100);

      
      ros::Subscriber sub=nh.subscribe("/robot_car/vel_cmd",100,doConmmand);

      ros::Rate rate=10;

      while(ros::ok) {
             t2=ros::Time::now();
            ROS_INFO("the command publish time:%.2f",t2.toSec());
            if (t2-t1>d)
            {
                  rrSpd.data=0;
                  rlSpd.data=0;
                  flStrPos.data=0;
                  frStrPos.data=0;
                  pub1.publish(rrSpd);
                  pub2.publish(rlSpd);
                  pub3.publish(flStrPos);
                   pub4.publish(frStrPos);
                  ROS_INFO("no command input, reset to 0");
            }
           else{
            pub1.publish(rrSpd);
            pub2.publish(rlSpd);
            pub3.publish(flStrPos);
            pub4.publish(frStrPos);
           }
            ros::spinOnce();
            rate.sleep();
      }
      ros::spin();
      return 0;
}