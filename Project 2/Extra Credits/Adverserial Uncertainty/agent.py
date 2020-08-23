# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
# This Adverserial Uncertainty Model is implemented in the TTC Method
# Code submitted by Huzefa Kagalwala (C48290423) and Rahil Modi (C14109603)
import numpy as np
import math
import random
from math import *
from scipy.spatial import distance as dist

class Agent(object):

    def __init__(self, csvParameters, ksi = 0.5, dhor = 10, timehor = 5, goalRadiusSq = 1, maxF = 10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        self.vel_uncertain = 0.2 # The velocity uncertainty constant
        self.nu = 0.1 # The radius of the disk from which velocity perturbations will be added
        
        # Calculating the random sample space for velocity perturbations
        N = 150
        cand_vel = np.zeros((N,2))
        vel_array = np.random.uniform(low = 0, high = (self.nu)**2+0.001, size = N)
        theta = np.random.uniform(low = 0, high = 2*(np.pi)+0.001, size = N) # Creating random angles
        sample_vel_x = np.sqrt(vel_array)*np.cos(theta) # Vx = sqrt(r)cos(theta)
        sample_vel_y = np.sqrt(vel_array)*np.sin(theta) # Vy = sqrt(r)sin(theta)
        cand_vel[:,0] = sample_vel_x
        cand_vel[:,1] = sample_vel_y
        self.eta = random.choice(cand_vel)
    
    # Function for the time to collison    
    def ttc(self, j):
        rad = self.radius + j.radius
        w = self.pos - j.pos
        c = np.dot(w, w) - (rad*rad)
        if c < 0:
            return 0
        v = (self.vel - j.vel + self.eta) - (self.vel_uncertain * (w / np.linalg.norm(w))) # Implementing the adverserial uncertainty model
        a = np.dot(v,v) - self.vel_uncertain**2
        b = np.dot(w,v) - (rad*self.vel_uncertain)
        if b > 0:
            return np.inf
        discr = (b*b) - (a*c)
        if discr <= 0:
            return np.inf
        tau = c / (-b + np.sqrt(discr))
        if tau < 0:
            return np.inf
        return tau
    

    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """
        Fg = (self.gvel - self.vel)/0.5
        Favoid = np.zeros_like(Fg)
        ttc_fin = np.inf
        for i in neighbors:
            if self.id != i.id and not i.atGoal:
                obj_sense_dist = dist.euclidean(self.pos, i.pos) - (self.radius + i.radius)
                if obj_sense_dist < 10:
                    ttc_val = self.ttc(i)
                    if ttc_val != np.inf:
                        if ttc_val < ttc_fin:
                            ttc_fin = ttc_val # This will compare all the ttc values within te neighbors
                            rel_pos = self.pos - i.pos
                            rel_vel = (self.vel - i.vel + self.eta) - (self.vel_uncertain * (rel_pos / np.linalg.norm(rel_pos))) # Implementing the adverserial uncertainty model
                            Favoid_dir = (rel_pos + (ttc_fin*rel_vel)) / (np.linalg.norm(rel_pos + (ttc_fin*rel_vel)))
                            Favoid_mag = max((5 - ttc_fin) , 0) / (ttc_fin + 1e-6)
                            Favoid = Favoid_mag*Favoid_dir                 
        self.F = Fg + Favoid
        self.F = np.clip(self.F, -10, 10) # Clipping the vector components to a magnitude of 10
        
    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt
            self.vel = np.clip(self.vel , -self.maxspeed, self.maxspeed) # Clipping the maximum velocity components to maxspeed
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed 
            
            
  