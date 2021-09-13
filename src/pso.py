#!/usr/bin/env python

import random
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point

prev_global=0
prev_val=[]
prev_val2=[]

def objective_function(x):
    
    total=math.sqrt(x[0]**2+x[1]**2)
    # y = total
    print(x)
    y = total+total*(random.randint(-2,2)/10)
    
    return y
 
a1=a2=50
bounds=[(-1000,1000),(1000,-1000)]  
nv = 2                  
mm = -1                  
particle_size=3       
iterations=20          
w=0.85                   
c1=1                  
c2=2                  

class Particle():
    global a1, a2
    def __init__(self,bounds):
        self.particle_position=[]                    
        self.particle_velocity=[]                     
        self.local_best_particle_position=[]          
        self.fitness_local_best_particle_position= initial_fitness  
        self.fitness_particle_position=initial_fitness             
         
        self.particle_position=[a1,a2]
        for i in range(nv):
            # self.particle_position.append(random.uniform(bounds[i][0],bounds[i][1])) # generate random initial position
            self.particle_velocity.append(random.uniform(-1,1)) # 
 
    def evaluate(self,objective_function):
        self.fitness_particle_position=objective_function(self.particle_position)
        if mm == -1:
            if self.fitness_particle_position < self.fitness_local_best_particle_position:
                self.local_best_particle_position=self.particle_position                  
                self.fitness_local_best_particle_position=self.fitness_particle_position  
        if mm == 1:
            if self.fitness_particle_position > self.fitness_local_best_particle_position:
                self.local_best_particle_position=self.particle_position                  
                self.fitness_local_best_particle_position=self.fitness_particle_position  
 
    def update_velocity(self,global_best_particle_position):
        for i in range(nv):
            r1=random.random()
            r2=random.random()
 
            cognitive_velocity = c1*r1*(self.local_best_particle_position[i] - self.particle_position[i])
            social_velocity = c2*r2*(global_best_particle_position[i] - self.particle_position[i])
            self.particle_velocity[i] = w*self.particle_velocity[i]+ cognitive_velocity + social_velocity
            print("VELOCITY: ")
            print(self.particle_velocity[i])

 
    def update_position(self,bounds):
        for i in range(nv):
            self.particle_position[i]=self.particle_position[i]+self.particle_velocity[i]
            print("position:")
            print(self.particle_position)

            if self.particle_position[i]>bounds[i][1]:
                self.particle_position[i]=bounds[i][1]

            if self.particle_position[i] < bounds[i][0]:
                self.particle_position[i]=bounds[i][0]
                 
total_time=[]
class PSO():

    def __init__(self,objective_function,bounds,particle_size,iterations):
        global prev_val
        fitness_global_best_particle_position=initial_fitness
        global_best_particle_position=[]
 
        swarm_particle=[]
        for i in range(particle_size):
            swarm_particle.append(Particle(bounds))
        A=[]
        desired_pos_pub = rospy.Publisher("/desired_pos",Point,10)
         
        for i in range(iterations):
            for j in range(particle_size):
                total_time.append(abs(prev_global-swarm_particle[j].fitness_particle_position)/20)

                swarm_particle[j].evaluate(objective_function)
 
                if mm ==-1:
                    if swarm_particle[j].fitness_particle_position < fitness_global_best_particle_position:
                        global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
                if mm ==1:
                    if swarm_particle[j].fitness_particle_position > fitness_global_best_particle_position:
                        global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
            for j in range(particle_size):
                swarm_particle[j].update_velocity(global_best_particle_position)
                swarm_particle[j].update_position(bounds)
                 
            A.append(fitness_global_best_particle_position)  
             
             
        print('Optimal solution:', global_best_particle_position)
        final_pos = Point()
        final_pos.x = global_best_particle_position[0]
        final_pos.y = global_best_particle_position[1]
        print(final_pos)
        
        while desired_pos_pub.get_num_connections == 0:
            rospy.sleep(1)
        
        #desired_pos_pub.publish(final_pos)


        print('Objective function value:', fitness_global_best_particle_position)


if mm == -1:
    initial_fitness = float("inf")
if mm == 1:
    initial_fitness = -float("inf") 
 

if __name__ =="__main__":
    #rospy.init_node("PSO")

    PSO(objective_function,bounds,particle_size,iterations)


#to_sum=[]
#to_sum = total_time[15:]

#sum_time = 0
##for i in range(len(total_time)):
 #   if total_time[i] != float('inf'):
  #      sum_time = sum_time+total_time[i]

#print('total time', sum_time)
