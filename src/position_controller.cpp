#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h>

using namespace std;

class Controller
{   
    public:

        ros::NodeHandle n;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber odom_sub;
        ros::Subscriber desired_pos_sub;

        geometry_msgs::Twist twist;
        //ros::Subscriber command_pose_sub;

        nav_msgs::Odometry odom;
        
        geometry_msgs::Point position;
        geometry_msgs::Point desired_position;
        geometry_msgs::Quaternion quaternion;
        double yaw_ =1;

        ros::Timer timer;
    

    
    

    //ros::Subscriber pose_sub;

    
        Controller(ros::NodeHandle *nh)
        {
            n = *nh;
            odom_sub = nh->subscribe("/odom", 1, &Controller::odomCallback, this);
            cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            desired_pos_sub = nh->subscribe("/desired_pos",10,&Controller::des_pos_cb,this);

            cout<<"conds";
            //command_pose_sub = n.subscribe("command_point", 1, &Controller::command_pose_clbk, this);
            //this->timer = nh->createTimer(ros::Duration(0.1), std::bind(&Controller::print_yaw, this, std::placeholders::_1));

        }
        
        
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {  
            odom = *msg;
            //ROS_INFO("Received ID: [%f]",odom.pose.pose.position.x);
            position = odom.pose.pose.position;
            quaternion = odom.pose.pose.orientation;
            double roll, pitch,yaw;
            tf::Matrix3x3(tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)).getRPY(roll, pitch, yaw_);
            
            cout << "Lmao" << endl;

        }
        void des_pos_cb(const geometry_msgs::Point::ConstPtr& msg)
        {
            desired_position = *msg;
            print_yaw(&desired_position);


        }
        void print_yaw(geometry_msgs::Point *desired_point)
        {   
            

            double err_pos=err_pos = sqrt(pow(desired_position.x - position.x, 2) + pow(desired_position.y - position.y, 2));
            double angle = atan2(desired_position.y - position.y, desired_position.x - position.x);
            cout<<angle;
            double err_yaw = angle - yaw_;
            cout<<err_yaw;

            err_yaw = angle - yaw_;

                //twist.angular.z = 0.4;

                //cmd_vel_pub.publish(twist);
            ROS_INFO("err_yaw: [%f]",err_yaw);
            ROS_INFO("err_pos: [%f]",err_pos);

        }
        
        void go_to_desired_point(geometry_msgs::Point *desired_point)
        {   
            double err_pos = sqrt(pow(desired_point->x - position.x, 2) + pow(desired_point->y - position.y, 2));
            double angle = atan2(desired_point->y - position.y, desired_point->x - position.x);
            
            ros::Rate r(10);

            while(abs(err_pos) > 0.1)
            {
                double err_yaw = angle -yaw_;
                if(abs(err_yaw)>0.2)
                {
                    fix_yaw(angle);

                }
                else
                    twist.linear.x = 0.2;
                cmd_vel_pub.publish(twist);


            }

            twist.angular.z = 0;
            twist.linear.x = 0;
            cmd_vel_pub.publish(twist);

        }
        void fix_yaw(double angle)  
        {   double err_yaw = angle-yaw_;
            while(abs(err_yaw)>0.2)
            {
                twist.angular.z = 0.2*err_yaw;
                err_yaw = angle - yaw_;
            }
        }
            

};
int main(int argc, char **argv)
{
    ros::init(argc,argv,"position_controller_node");
    //initialize node handle

    ros::NodeHandle nh;
    //pass node handle to class
    Controller controller(&nh);
    geometry_msgs::Point pos;
    pos.x =1;
    // controller.print_yaw();
    
    
    ros::spin();
    
}
