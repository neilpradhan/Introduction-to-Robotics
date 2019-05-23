#! /usr/bin/env python3

"""
    # {student full name} NEIL PRADHAN
    # {student email} npradhan@kth.se
"""
import math
import numpy as np

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    x1 = x-0.07
    l1= 0.3
    l2 = 0.35   
    c2 = (x1**2+y**2-l1**2-l2**2)/(2*l1*l2)
    s2 = math.sqrt(1-c2**2)
    theta_2 = math.acos(c2)
    

    
    cos_psi = (x1**2+y**2+l1**2-l2**2)/(2*l1*math.sqrt(x1**2+y**2))
    psi = math.acos(cos_psi)
    #print(psi,theta_2)
    

    beta = math.atan2(y,x1)
    theta_1 = beta+psi

    

    q= [theta_1,-theta_2,z]


  
    return q


    
def homogeneous(q):
    T_wrt_base = np.asarray([[1,0,0,0],[0,1,0,0],[0,0,1,0.311],[0,0,0,1]])
    T7_wrt_end = np.asarray([[1,0,0,0],[0,1,0,0],[0,0,1,0.078],[0,0,0,1]])
    B=[] #list of all T's like B[0] =T[0]  and so on
    l=np.identity(4)
    alpha = [math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,math.pi/2,-math.pi/2,0]
    d=[0,0,0.4,0,0.39,0,0]
    for i in range(7):
        T = [[math.cos(q[i]),-math.sin(q[i])*math.cos(alpha[i]),math.sin(q[i])*math.sin(alpha[i]),0],[math.sin(q[i]),math.cos(q[i])*math.cos(alpha[i]), -math.cos(q[i])*math.sin(alpha[i]),0],[0,math.sin(alpha[i]),math.cos(alpha[i]),d[i]],[0,0,0,1]]       
        T = np.asarray(T)
        B.append(T)
        l=np.dot(l,T)       
    l=np.dot(T_wrt_base,l)
    l=np.dot(l,T7_wrt_end) #l is the total homogeneous matrix multiplication of all matrices      
    #print(np.shape(l))
    return l


#H is the total homogeneous matrix
#calculated postion has x,y,z

def calculated_postitions(H):
        pt = np.array([[0],[0],[0],[1]])
        K=np.dot(H,pt)
        K = K[0:3,0]
        K =np.reshape(K,(3,1))
        return K

def calculated_rotation(H):
        m = H[0:3,0:3]
        m=m.flatten()
        m = m.reshape(9,1)
        # that is 9 by 1 rotation matrix
        return m

def Total_x_calculated(m,k):
        v=np.vstack((m,k))
        #l is 12 by 1 matrix has 9 rotation terms and three translation terms
        return v


def Jacobian(q):
       #12 by 1 matrix
        l=homogeneous(q)
        k=calculated_postitions(l)
        m=calculated_rotation(l)
        X_calculated = Total_x_calculated(m,k)# matrix of 12 by 1
       
        h=0.00000000000001
        q1=q.copy()
       
        q1[0] = q1[0]+h
        l_h=homogeneous(q1)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P1=(X_h - X_calculated)/h #matrix of 12 by 1
       
        q2=q.copy()
       
        q2[1] = q2[1]+h
        l_h=homogeneous(q2)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P2=(X_h - X_calculated)/h #matrix of 12 by 1
        
        q3=q.copy()
       
        q3[2] = q3[2]+h
        l_h=homogeneous(q3)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P3=(X_h - X_calculated)/h #matrix of 12 by 1

        q4=q.copy()
       
        q4[3] = q4[3]+h
        l_h=homogeneous(q4)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P4=(X_h - X_calculated)/h #matrix of 12 by 1
        

        q5=q.copy()
       
        q5[4] = q5[4]+h
        l_h=homogeneous(q5)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P5=(X_h - X_calculated)/h #matrix of 12 by 1       
    


        q6=q.copy()
       
        q6[5] = q6[5]+h
        l_h=homogeneous(q6)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P6=(X_h - X_calculated)/h #matrix of 12 by 1   
    
        q7=q.copy()
       
        q7[6] = q7[6]+h
        l_h=homogeneous(q7)
        k_h=calculated_postitions(l_h)
        m_h=calculated_rotation(l_h)
        X_h = Total_x_calculated(m_h,k_h)# matrix of 12 by 1
       
        P7=(X_h - X_calculated)/h #matrix of 12 by 1      
    
        J = [P1[:,0],P2[:,0],P3[:,0],P4[:,0],P5[:,0],P6[:,0],P7[:,0]] # 12 by 7
        J = np.transpose(J)
        return J


             
def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q_initial = joint_positions #it must contain 7 elements
    q_initial = np.asarray(q_initial)
    q_initial =np.reshape(q_initial,(7,1))
    R = np.asarray(R)
    X_des = [R[0,0],R[0,1],R[0,2],R[1,0],R[1,1],R[1,2],R[2,0],R[2,1],R[2,2],x,y,z]
    X_des= np.asarray(X_des)
    X_des =np.reshape(X_des,(12,1))

        #print((X_des))
    q=q_initial #initial approximation
      
    dis = 100 
    #while dis >5:
    for i in range(900):
        #q=np.asarray(q)    
        l=homogeneous(q)
        #print(np.linalg.norm(q))
        k=calculated_postitions(l)
        m=calculated_rotation(l)
            
        X_calculated = Total_x_calculated(m,k)# matrix of 12 by 1
        J = Jacobian(q)#12 by 7
        E=np.linalg.pinv(J)# 7 by 12
        #print(J)
        W = X_des - X_calculated
        #print(X_des[9:12]) 
        delta_q = np.dot(E,W)# check multiplying oposite
        #print(delta_q)
        q = q_initial+delta_q
        #q = q.tolist()
        #print(q)
        #print(np.linalg.norm(q))
        #dis = np.linalg.norm(W)
        #print(dis)
        q11 = np.squeeze(q) 
        return q11
