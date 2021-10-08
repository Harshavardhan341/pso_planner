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
        double yaw_;
        double err_yaw,err_pos;

        ros::Timer timer;
        
    

    
    

    //ros::Subscriber pose_sub;

    
        Controller(ros::NodeHandle *nh)
        {
            n = *nh;
            odom_sub = nh->subscribe("odom", 1, &Controller::odomCallback, this);
            cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
            desired_pos_sub = nh->subscribe("desired_pos",10,&Controller::des_pos_cb,this);
            

            //timer = nh->createTimer(ros::Duration(0.1),&Controller::odomCallback,this);
            //command_pose_sub = n.subscribe("command_point", 1, &Controller::command_pose_clbk, this);

            
        }
        
        
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {  
            odom = *msg;
            //ROS_INFO("Received ID: [%f]",odom.pose.pose.position.x);
            position = odom.pose.pose.position;
            quaternion = odom.pose.pose.orientation;
            double roll, pitch,yaw;
            tf::Matrix3x3(tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)).getRPY(roll, pitch, yaw);
            this->yaw_ = yaw;
            

        }
        void des_pos_cb(const geometry_msgs::Point::ConstPtr& msg)
        {
            desired_position = *msg;
            this->go_to_desired_point(&desired_position);
        }

        void fix_yaw(double angle)
        {   
            
            
            ros::spinOnce();

            
            
            err_yaw = angle - this->yaw_;
            double prev_err = 0;
            ros::Rate r(10);
            while(fabs(err_yaw) >= 0.05)
            {   ros::spinOnce();
                

                //ROS_INFO("err_yaw: [%f]",err_yaw);

                twist.angular.z = 1.5*err_yaw - 2*(err_yaw - prev_err);
                
                prev_err = err_yaw;
                err_yaw = angle - this->yaw_;

                cmd_vel_pub.publish(twist);
                r.sleep();
            }
            

            twist.angular.z = 0;

            cmd_vel_pub.publish(twist);
            //ROS_INFO("err_yaw: [%f]",err_yaw);
            

        }
        
        void go_to_desired_point(geometry_msgs::Point *desired_point)
        {   



            err_pos = sqrt(pow(desired_point->x - position.x, 2) + pow(desired_point->y - position.y, 2));
            double angle = atan2(desired_point->y - position.y, desired_point->x - position.x);
            err_yaw = angle - this->yaw_;
            ros::Rate rate(10);
            while(fabs(err_pos)>0.17)            
            {   ros::spinOnce();
            
                //ROS_INFO("err_pos: [%f]",err_pos);
                //ROS_INFO("err_yaw: [%f]",err_yaw);

                
                if(fabs(err_yaw)>0.05)
                {   
                    cout<<"should come here"<<endl;                    
                    fix_yaw(angle);
                }
                
                twist.linear.x = 0.3;
                cmd_vel_pub.publish(twist);
                err_pos = sqrt(pow(desired_point->x - position.x, 2) + pow(desired_point->y - position.y, 2));
                err_yaw = angle - this->yaw_;


                rate.sleep();
            }
            twist.linear.x = 0;
            cmd_vel_pub.publish(twist);

            

            



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

    ros::spin();
    
    
    
    
}
