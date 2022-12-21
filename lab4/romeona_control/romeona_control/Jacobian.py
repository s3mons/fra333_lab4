import numpy as np
import math

def forward_kin(q):
    h = 0.135
    l1 = 0.04
    l2 = 0.15
    l3 = 0.12
    DH_table = ([[0,0,h,0.],
                [l1,math.pi/2,0.,0.],
                [l2,0.,0.,0.]])
    P = [1,1,1]
    H = np.identity(4)
    joint = 3
    H3_e = [[0.,0.,1.,l3],
            [1.,0.,0.,0.],
            [0.,1.,0.,0.],
            [0.,0.,0.,1.]]

    Rotation = np.empty((3,3,4),dtype=np.float32)
    Position = np.empty((1,3,4)) 

    for i in range(0,joint):
        Tx = DH_table[i][0] 
        Rx = DH_table[i][1] 
        Tz = DH_table[i][2] 
        Rz = DH_table[i][3] 
        Tra_x   = [[1.,0.,0.,Tx],
                [0.,1.,0.,0.],
                [0.,0.,1.,0.],
                [0.,0.,0.,1.]]

        Rot_x   = [[1.,0.,0.,0.],
                [0.,math.cos(Rx),-1*math.sin(Rx),0.],
                [0.,math.sin(Rx),math.cos(Rx),0.],
                [0.,0.,0.,1.]]
                            
        Tra_z   = [[1.,0.,0.,0.],
                [0.,1.,0.,0.],
                [0.,0.,1.,Tz],
                [0.,0.,0.,1.]]
        Rot_z   = [[math.cos(Rz),-1*math.sin(Rz),0.,0.],
                [math.sin(Rz),math.cos(Rz),0.,0.],
                [0.,0.,1.,0.],
                [0.,0.,0.,1.]]

        if P[i] == 1:
            # print(q[i])
            Rot_z_q   = [[math.cos(q[i]),-1*math.sin(q[i]),0.,0.],
                        [math.sin(q[i]),math.cos(q[i]),0.,0.],
                        [0.,0.,1.,0.],
                        [0.,0.,0.,1.]]
            Hj = Rot_z_q
        else:
            Tra_z_q   = [[1.,0.,0.,0.],
                        [0.,1.,0.,0.],
                        [0.,0.,1.,q[i]],
                        [0.,0.,0.,1.]]
            Hj = Tra_z_q

        H = H @ Tra_x @ Rot_x @ Tra_z @ Rot_z @ Hj
        Rotation[:,:,i] = np.array(H[:3,:3])
        Position[:,:,i] = [H[:3,3]]
 
        
    H0_e = H@H3_e
    Rotation[:,:,3] = np.array(H0_e[:3,:3])
    Position[:,:,3] = H0_e[:3,3]

    R_e = Rotation[:,:,3]
    p_e = Position[:,:,3]
    #print(Tra_x,"\n",Rot_x,"\n",Tra_z,"\n",Rot_z)
    # print(p_e - np.array([1,1,1]))
    # print(Position[:,:,1])
    # print(p_e)
    return Rotation,Position,R_e,p_e,H0_e

forward_kin([1.57,0,0])

def Jacobian(q):
    J = np.zeros((6,len(q))); # สร้าง Matrix ที่มีขนาด 6*3 
    R,P,R_e,p_e,H0_e= forward_kin(q)

    for i in range(len(q)): 
        Linear = np.cross(R[:,:,i][:,2] ,p_e-P[:,:,i]) 
                            
        Rotational = R[:,:,i][:,2] #

        J[0:3,i] = Rotational
        J[3:,i] = Linear
    J = J.tolist() # เปลี่ยนจาก np.array เป็น list

    return J

def pos_inverse_kinematics(p):
    # setup parameters
    flag = True
    h = 0.135
    l_1 = 0.04
    l_2 = 0.15
    l_3 = 0.12

    x = p[0]
    y = p[1]
    z = p[2]
    g1 = 1
    g2 = -1
    
    # Q1
    s_1 = y/g1
    c_1 = x/g1
    q_1 = np.arctan2(s_1,c_1)

    # Q3
    c_3 = ((g1*((x**2+y**2)**0.5) - l_1)**2 + (z-h)**2 - l_2**2 - l_3**2)/(2*l_2*l_3)
    # c_3 = round(c_3,13)
    
    s_3 = g2*((1-c_3**2)**0.5)
    q_3 = np.arctan2(s_3,c_3)
  
        
    #Q2
    c_2 = (l_2 + l_3*c_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_3*s_3)*(z-h)
    s_2 = (-l_3*s_3)*(g1*((x**2+y**2)**0.5)-l_1) + (l_2 + l_3*c_3)*(z-h)
    q_2 = np.arctan2(s_2,c_2)

    return [q_1,q_2,q_3]

def vel_inverse_kinematics(q,p_dot):
    A = np.array(Jacobian(q))
    J = A[3:]
    J_inv = np.linalg.inv(J)
    q_dot = J_inv @ p_dot
    return q_dot.tolist()

# print(np.subtract(np.array([1,2,3]),np.array([3,2,3])))
