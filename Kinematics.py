##Forward Kinematics

import numpy as np


def get_dh_table(q_current):

    dh_table = np.array([[q_current[0],np.pi/2,0,0.08916],\
                             [q_current[1] +np.pi/2,0,0.425,0],\
                             [q_current[2],0,0.3923,0],\
                             [q_current[3] -np.pi/2,-np.pi/2,0,0.1091],\
                             [q_current[4],+np.pi/2,0,0.0947],\
                             [q_current[5],0,0,0.0823]])
    
    return dh_table

# function to get the transformation between frame{i} to frame{i-1} 
def link_transformation_matrix(dh_line):
    theta = dh_line[0]; alpha = dh_line[1]; a = dh_line[2]; d = dh_line[3]
    c_th = np.cos(theta); s_th = np.sin(theta); c_alp = np.cos(alpha); s_alp = np.sin(alpha)
    
    T=np.array([[c_th,-s_th*c_alp,s_th*s_alp,a*c_th],[s_th,c_th*c_alp,-c_th*s_alp,a*s_th],[0,s_alp,c_alp,d],[0,0,0,1]])
    return np.round(T,3)

#find all transformations between frame{i} to frame{i-1} and put it in a list
def link_transformations(dh_table):
    #list to contain all frame transformations from frame{i} to frame {i-1}
    transformations = np.array([np.eye(4) for i in range(len(dh_table))])
    
    #get all link transformations
    for i in range(len(dh_table)):
        transformations[i] = link_transformation_matrix(dh_table[i])

    return transformations

#find transformations from each frame to base frame
def to_base_frame(dh_table):
    #list to contain all transformations from frame {i-1} to frame {0}
    Ti_1_0 = np.array([np.eye(4) for i in range(len(dh_table) + 1)])
    
    transformations = link_transformations(dh_table)
    
    #get transformations from each frame to base frame
    for i in range(len(transformations)):
        Ti_1_0[i+1] = Ti_1_0[i]@link_transformation_matrix(dh_table[i])
        
    return Ti_1_0



##Inverse Kinematics

# get elements of a skew-symmetric matrix
def vec(matrix):
    
    v = np.zeros(3)
    
    v[0] = matrix[2,1]
    v[1] = matrix[0,2]
    v[2] = matrix[1,0]
    
    return v


# find jacobian matrix for any robot
def jacobian(dh_table,joint_types):
    
    Jp = np.zeros((3,len(dh_table))) #array to contain position jacobian
    Jr = np.zeros((3,len(dh_table))) #array to contain orientation jacobian
    
    Ti_1_0 = to_base_frame(dh_table) #transformations from each frame to base frame
    
    #get Position and Orientation Jacobians
    for i in range(len(dh_table)):
        #check if joint is revolute or prismatic
        if joint_types[i] == 'revolute': 
            Jp[:,i] = np.cross(Ti_1_0[i][0:3,2],(Ti_1_0[-1][0:3,3]-Ti_1_0[i][0:3,3]))
            Jr[:,i] = Ti_1_0[i][0:3,2]
        elif joint_types[i] == 'prismatic':
            Jp[:,i] = Ti_1_0[i][0:3,2]
            Jr[:,i] = 0
        else:
            return print("Invalid joint type")
    
    #arrange position and Orientation Jacobians into a 6xn matrix
    J = np.zeros((6,len(dh_table)))
    
    for i in range(len(dh_table)):
        J[0:3,i] = Jp[:,i]
        J[3:6,i] = Jr[:,i]
    
    return J.round(3)


#find the discrete velocity
def dx(current_pose,desired_pose):
    
    x_current = current_pose[0,3]
    y_current = current_pose[1,3]
    z_current = current_pose[2,3]
    R_current = current_pose[0:3,0:3]
    
    x_d = desired_pose[0,3]
    y_d = desired_pose[1,3]
    z_d = desired_pose[2,3]
    R_d = desired_pose[0:3,0:3]
    
    w = np.array(vec((R_d@np.transpose(R_current)) - np.eye(3)))
    
    delta_X = np.array([x_d - x_current,y_d - y_current,z_d - z_current,w[0],w[1],w[2]])
    
    return delta_X

#Apply Newton-Raphson Method to find desired joint angles
def ik(q_current,desired_pose,joint_types):

    dh_table = get_dh_table(q_current)
    current_jacobian = jacobian(dh_table,joint_types)

    current_pose = to_base_frame(dh_table)[-1]

    delta_X =dx(current_pose,desired_pose)

    q_current = q_current + (np.linalg.pinv(current_jacobian)@delta_X).round(3)
    error = np.sqrt(np.dot(delta_X,delta_X))
    
    return q_current , error
