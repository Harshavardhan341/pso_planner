#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"



#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std;

class Position_controller : public rclcpp::Node
{
    public: 
        Position_controller():Node("position_controller")
        {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/box_bot1/odom",10,std::bind(&Position_controller::odom_cb,this,std::placeholders::_1));
            pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>("/command/pose2d",10,std::bind(&Position_controller::command_pose2d_cb,this,std::placeholders::_1));
            twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/box_bot1/cmd_vel",100);
        }
    private:
        void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
        {   
            odom = *msg;
            geometry_msgs::msg::Quaternion q = msg->pose.pose.orientation;
            yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
            cout<<"Yaw: "<<yaw<<endl;



            

            
        }
        void command_pose2d_cb(const geometry_msgs::msg::Pose2D::SharedPtr msg)
        {
            desired_pose=*msg;
            
            
            
            
                position_control(&desired_pose,&odom);
                
            
            
        }
        void position_control(geometry_msgs::msg::Pose2D *desired_pose_,nav_msgs::msg::Odometry *odom_)
        {
            double angle_to_goal=atan2(desired_pose_->y-odom_->pose.pose.position.y,desired_pose_->x-odom_->pose.pose.position.x);
            while(abs(err_pos(&desired_pose,&odom))>0.1)
            {
                twist.angular.z = 0.1*err_yaw(angle_to_goal,yaw);
                twist.linear.x = 0.5*err_pos(&desired_pose,&odom);
                twist_pub->publish(twist);

            }


        }
        double err_yaw(double desired_yaw,double yaw_)
        {
            double err_yaw = desired_yaw-yaw_;
            return err_yaw;
        }
        double err_pos(geometry_msgs::msg::Pose2D *desired_pose_,nav_msgs::msg::Odometry *odom_)
        {
            double err_pos;
            err_pos = sqrt(pow(desired_pose_->x-odom_->pose.pose.position.x,2)+pow(desired_pose_->y-odom_->pose.pose.position.y,2));
            return err_pos;
        }
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
        geometry_msgs::msg::Quaternion q;
        geometry_msgs::msg::Pose2D desired_pose;
        nav_msgs::msg::Odometry odom;
        geometry_msgs::msg::Twist twist;
        double yaw;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Position_controller>());
  rclcpp::shutdown();
  return 0;
}