#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <bits/stdc++.h>

class Particle
{
    public:
        geometry_msgs::Point global_best,local_best_pos,current_pos,future_pos,goal;
        
        float w,c1,c2,r1,r2;
        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        float fitness_particle_position = std::numeric_limits<float>::infinity();
        float local_best_fitness = std::numeric_limits<float>::infinity();
        float global_best_fitness,global_best_x,global_best_y;
        

        Particle(ros::NodeHandle *nh)

        {
            odom_sub = nh->subscribe("odom",1,&Particle::odomCallback,this);
            goal_sub = nh->subscribe("/goal",1,&Particle::goalCallback,this);
        } 
        void goalCallback(const geometry_msgs::Point::ConstPtr &msg)
        {
            goal = *msg;

        }
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            current_pos = msg->pose.pose.position;

        }
        float costfunction(geometry_msgs::Point *p)
        {
            float dist = sqrtf(pow((goal.x - p->x),2)+pow((goal.y - p->y),2));
            return dist;

        }
        void evaluate()
        {   
            this->fitness_particle_position = costfunction(&current_pos);
            if(this->fitness_particle_position < this->local_best_fitness)
            {
                this->local_best_pos = this->current_pos;
                this->local_best_fitness = this->fitness_particle_position;
            }
        }
        void future_pos(ros::NodeHandle *nh)
        {   float nr,count;
            
            nh->getParam("/global_best",global_best_fitness);
            nh->getParam("/count",count);
            nh->getParam("/nr",nr);
            if(count <= nr)
            {
                if(this->local_best_fitness < global_best_fitness)
                    nh->setParam("/global_best",this->local_best_fitness);
                    //set global_best_position
                nh->setParam("/count",++count);                 

            }
            else 
                nh->getParam("/global_best",global_best_fitness);
                
            
            

        }



};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Particle");
    ros::NodeHandle nh;

}