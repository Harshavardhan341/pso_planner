#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <bits/stdc++.h>

using namespace std;

class Particle
{
    public:
        geometry_msgs::Point global_best_pos,local_best_pos,current_pos,future_position,goal;
        //current_pos,future_pos,goal;
        
        float w,c1,c2,r1,r2;
        ros::Subscriber odom_sub;
        ros::Subscriber goal_sub;
        float fitness_particle_position = std::numeric_limits<float>::infinity();
        float local_best_fitness = std::numeric_limits<float>::infinity();
        float global_best_fitness;
        geometry_msgs::Twist particle_velocity;
        ros::NodeHandle n;
        ros::Publisher command_pos_pub;

        Particle(ros::NodeHandle *nh)

        {   this->n = *nh;
            //random number between -1 and 1
            std::default_random_engine gen;
            
            c1 = 1;
            c2 = 2;
            w = 0.85;
            command_pos_pub = this->n.advertise<geometry_msgs::Point>("desired_pos",10);
            odom_sub = this->n.subscribe("odom",1,&Particle::odomCallback,this);
            goal_sub = this->n.subscribe("/goal",1,&Particle::goalCallback,this);
        } 
        void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
        {
            goal = *msg;
            cout<<"Goal_x: "<<goal.x;
            cout<<"Goal_y: "<<goal.y;
            for(int i=0;i<20;i++)
            {
                this->future_pos();
            }
            

        }
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {   
            this->current_pos = msg->pose.pose.position;
            this->particle_velocity = msg->twist.twist;

            

        }
        float costfunction(geometry_msgs::Point *p)
        {
            float dist = sqrtf(pow((this->goal.x - p->x),2)+pow((this->goal.y - p->y),2));
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
        public:


        
        void future_pos()
        {   float nr,count;
            evaluate();
            this->n.getParam("/global_best/fitness",global_best_fitness);
            this->n.getParam("/count",count);
            
            this->n.getParam("/nr",nr);
            if(count <= nr)
            {
                if(this->local_best_fitness < global_best_fitness)
                    
                    setGlobalBest(this->local_best_fitness,this->local_best_pos);
                
                if(count<nr)
                    {this->n.setParam("/count",++count); 
                    ros::Duration(2.0).sleep();
                     } 
                else 
                    count = 0;
                    this->n.setParam("/count",0);     
                

            }        
            getGlobalBest(this->global_best_fitness,this->global_best_pos);
            cout<<"Global_best fitness: "<<this->global_best_fitness<<"\n";
            this->future_position = update_position(global_best_pos);
            //command_pos_pub.publish(future_position);

            
        }
        geometry_msgs::Point update_position(geometry_msgs::Point& global_best_position)
        {   
            geometry_msgs::Point future_position_;

            this->r1 = ((double) rand() / (RAND_MAX)) + 1;
            this->r2 = ((double) rand() / (RAND_MAX)) + 1;

            geometry_msgs::Point cognitive_velocity,social_velocity;
            cognitive_velocity.x = c1*r1*(this->local_best_pos.x-this->current_pos.x);
            cognitive_velocity.y = c1*r1*(this->local_best_pos.y-this->current_pos.y);
            
            social_velocity.x = c2*r2*(global_best_position.x-this->current_pos.x);
            social_velocity.y = c2*r2*(global_best_position.y-this->current_pos.y);

            particle_velocity.linear.x = w*particle_velocity.linear.x + cognitive_velocity.x + social_velocity.x;
            particle_velocity.linear.y = w*particle_velocity.linear.y + cognitive_velocity.y + social_velocity.y;
            
            future_position_.x = this->current_pos.x + this->particle_velocity.linear.x;
            future_position_.y = this->current_pos.y + this->particle_velocity.linear.y;

            cout<<"Future pos x: "<<ros::this_node::getNamespace()<<": "<<future_position_.x<<"\n";
            cout<<"Future pos y: "<<ros::this_node::getNamespace()<<": "<<future_position_.y<<"\n";
            
            return future_position_;
            
            
            
        }



};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"Particle");
    ros::NodeHandle nh;
    Particle particle(&nh);
    int i =0;
    ros::spin();

}