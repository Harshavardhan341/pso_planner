#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <bits/stdc++.h>

class Particle
{
    public:
        geometry_msgs::Point global_best_pos,local_best_pos,current_pos,future_pos,goal;
        //current_pos,future_pos,goal;
        
        float w,c1,c2,r1,r2;
        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        float fitness_particle_position = std::numeric_limits<float>::infinity();
        float local_best_fitness = std::numeric_limits<float>::infinity();
        float global_best_fitness;
        geometry_msgs::Twist particle_velocity;
        ros::NodeHandle n;

        Particle(ros::NodeHandle *nh)

        {   this->n = *nh;
            //random number between -1 and 1
            std::default_random_engine gen;
            std::uniform_real_distribution<float> dist(-1,1);


            this->particle_velocity.linear.x = dist(gen);
            this->particle_velocity.linear.y = dist(gen);
            odom_sub = this->n.subscribe("odom",1,&Particle::odomCallback,this);
            goal_sub = this->n.subscribe("/goal",1,&Particle::goalCallback,this);
        } 
        void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
        {
            goal = *msg;
            //for 
            //future_pos()

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
        {   ros::spinOnce();
            this->fitness_particle_position = costfunction(&current_pos);
            if(this->fitness_particle_position < this->local_best_fitness)
            {
                this->local_best_pos = this->current_pos;
                this->local_best_fitness = this->fitness_particle_position;
            }
        }
        void setGlobalBest(const float local_best_fitness_,const geometry_msgs::Point& local_best_pos_)
        {  
                this->n.setParam("/global_best/fitness",local_best_fitness_);
                this->n.setParam("/global_best/x",local_best_pos_.x);
                this->n.setParam("/global_best/y",local_best_pos_.y);            
        }
        void getGlobalBest(float& global_best_fitness_,geometry_msgs::Point& global_best_pos_)
        {  
                this->n.getParam("/global_best/fitness",global_best_fitness_);
                this->n.getParam("/global_best/x",global_best_pos_.x);
                this->n.getParam("/global_best/y",global_best_pos_.y);            
        }
        
        void future_pos()
        {   float nr,count;
            
            this->n.getParam("/global_best/fitness",global_best_fitness);
            this->n.getParam("/count",count);
            this->n.getParam("/nr",nr);
            if(count <= nr)
            {
                if(this->local_best_fitness < global_best_fitness)
                    
                    setGlobalBest(this->local_best_fitness,this->local_best_pos);
                    //set global_best_position
                this->n.setParam("/count",++count);                 

            }
            else if(count > nr)
                getGlobalBest(this->global_best_fitness,this->global_best_pos); 
                        
            
            

        }
        geometry_msgs::Point update_position(geometry_msgs::Point& global_best_position)
        {
            this->r1 = ((double) rand() / (RAND_MAX)) + 1;
            this->r2 = ((double) rand() / (RAND_MAX)) + 1;

            
        }



};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Particle");
    ros::NodeHandle nh;

}