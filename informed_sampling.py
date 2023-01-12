import numpy as np

def sampling_sphere():
    sample = [2*np.random.random_sample()-1,2*np.random.random_sample()-1,2*np.random.random_sample()-1]
    if (sample[0]**2 + sample[1]**2 + sample[2]**2 < 1.0):
        return sample
    return sampling_sphere()


def informed_sample(x_start,x_goal,c_best):
    #prepare
    c_min = np.linalg.norm(x_start-x_goal)
    x_centre = (x_start+x_goal)/2
    
    #step1: rotation to world frame
    a1 = np.array([(x_goal[0]-x_start[0])/(c_min**2), (x_goal[1]-x_start[1])/(c_min**2), (x_goal[2]-x_start[2])/(c_min**2)])
    id1_t = np.array([1,0,0])
    M = np.outer(a1,id1_t) #get a 3x3 mtx by taking outer product
    #get U and V
    U, s, V = np.linalg.svd(M, full_matrices=True)
    #get rotation mtx C
    D = np.diag([1,1,np.linalg.det(U)*np.linalg.det(V)])
    Vt = np.array(V).T
    
    #rotation matrix from ellipsoid frame to world frame
    C = np.matmul(np.matmul(U,D),Vt)
    print("C=",C)   


    #step2unified sample in ball radius=1
    r = 1#np.random.random_sample()
    theta = 2 * np.pi * np.random.random_sample()
    phi = np.arccos(1 - 2 * np.random.random_sample())
    x = r*np.sin(phi) * np.cos(theta)
    y = r*np.sin(phi) * np.sin(theta)
    z = r*np.cos(phi)

    sample = sampling_sphere()
    new_node = sample

    r1=c_best/2
    r2=np.sqrt(c_best**2-c_min**2)/2
    r3=r2
    L=np.diag([r1,r2,r3])
    
    #step3 transform unified sample into world frame
    ellipsoid= np.matmul(np.matmul(C,L),sample)
    node=[0,0,0]
    node[0]=ellipsoid[0]+x_centre[0]
    node[1]=ellipsoid[1]+x_centre[1]
    node[2]=ellipsoid[2]+x_centre[2]

    new_node=[0,0,0]
    new_node[0]=node[0]
    new_node[1]=node[1]
    new_node[2]=node[2]
    return new_node