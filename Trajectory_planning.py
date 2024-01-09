import numpy as np
import math

#import zmqRemoteApi and make an instance of the RemoteAPIClient class
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

    
def generate_path(initial_pose, final_pose, dt, T):
    
    x_initial = initial_pose[0,3]
    y_initial = initial_pose[1,3]
    z_initial = initial_pose[2,3]
    
    #get initial Euler Angles
    
    alpha_initial , beta_initial, gamma_initial = sim.getEulerAnglesFromMatrix(initial_pose.flatten().tolist())
    
    x_final = final_pose[0,3]
    y_final = final_pose[1,3]
    z_final = final_pose[2,3]
    
    #get final Euler Angles
    
    alpha_final , beta_final, gamma_final = sim.getEulerAnglesFromMatrix(final_pose.flatten().tolist())
    
    
    t = 0
    
    x_array = []
    y_array = []
    z_array = []
    
    alpha_array = []
    beta_array = []
    gamma_array = []

    t_array = []    

    while(t<=T):
        s = t/T
        x = round(x_initial + (x_final - x_initial)*s,3)
        y = round(y_initial + (y_final - y_initial)*s,3)
        z = round(z_initial + (z_final - z_initial)*s,3)
        
        alpha = round(alpha_initial + (alpha_final - alpha_initial)*s,3)
        beta = round(beta_initial + (beta_final - beta_initial)*s,3)
        gamma = round(gamma_initial + (gamma_final - gamma_initial)*s,3)
        
        x_array.append(x)
        y_array.append(y)
        z_array.append(z)
        t_array.append(t)
        
        alpha_array.append(alpha)
        beta_array.append(beta)
        gamma_array.append(gamma)
        
        t = t+dt
        
    poses = np.array([np.zeros([3,4]) for i in range(len(x_array))])
    
    for i in range(len(x_array)):
        
        #build transformation matrix using position and euler angles
        k = np.array([sim.buildMatrix([x_array[i],y_array[i],z_array[i]],[alpha_array[i],beta_array[i],gamma_array[i]])]).reshape(3,4).round(3)
        poses[i] = k
        
    return poses

    