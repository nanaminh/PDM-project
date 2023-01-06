#######
# RO47005 Planning and decision making 22/23
# Group:10
# Aurthor: Danning Zhao
# email: D.ZHAO-3@student.tudelft.nl
#######

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
def getCoeff(k,n_order,ts):
    """
    Args:
        k (int): the order of derivative
        n_order (int): order of polynomial
        ts (0~1): time

    Returns:
        list: dim(1,n_order+1)
    """
    coeff=[0]*(n_order+1)
    if k==0:
        for i in range(0,n_order+1):
            coeff[i]=ts**i
    else:
        for i in range(k,n_order+1):#order number
            coeff[i]=np.math.factorial(i)/np.math.factorial(i-k)*ts**(i-k)
    
    return coeff

def getConstrainMtx(start_pos,goal_pos,n_order):
    """
    return the constraint matrix, for single coordinate
    Args:
        start_pos (list): start point of each sgment dim(1,n)
        goal_pos (list): goal point of each sgment dim(1,n)
        n_order (int): order of polynomial

    Returns:
        np.array: the constraint matrix
    """
    if len(start_pos)!=len(goal_pos):
        raise ValueError("The dimension of input start and goal list not same")
    
    mtxA=np.zeros((len(goal_pos)*(n_order+1),len(goal_pos)*(n_order+1)))
    mtxb=np.zeros(len(goal_pos)*(n_order+1))
    #########################for Amtx####################################
    #1. waypoints constraint
    #p i (S i−1 ) = w i−1 and p i (S i ) = w i for all i = 1, . . . , n (2*len(segment) constraints)
    for k in range(0,len(start_pos)):#index number
        mtxA[k,k*(n_order+1):(k+1)*(n_order+1)]=getCoeff(0,n_order,0)
        mtxA[k+len(start_pos),k*(n_order+1):(k+1)*(n_order+1)]=getCoeff(0,n_order,1)
        
    #2.for start and goal, the derivative 1,2,3=0
    #p 1 (S 0 ) = p (k)n (S n ) = 0 for all k = 1, . . . , 3 (6 constraints)
    for k in range(1,4):#derivative 1,2,3
        # print(np.shape(mtxA[2*len(start_pos)+k,0:7]))
        # print(np.shape(mtxA[2*len(start_pos)+3+k,-(n_order+1):]))
        # print(np.shape(getCoeff(n_order,k,0)))
        # print(np.shape(getCoeff(n_order,k,1)))
        mtxA[2*len(start_pos)+k-1,0:n_order+1] = getCoeff(k,n_order,0)
        mtxA[2*len(start_pos)+3+k-1,-(n_order+1):] = getCoeff(k,n_order,1)
        
    #3. continuity
    #p i (S i ) = p i+1 (S i ) = 0 for all k = 1, . . . , 6 (6*len(segment) − 6 constraints)
    for n in range(2,len(start_pos)+1):
        for k in range(1,7):#1-6
            # print("n",n)
            # print("k",k)
            #mtxA[2*len(start_pos)+6+(n-2)*6+k, (n-2)*(n_order+1)+1:(n*(n_order+1))] = [getCoeff(k,n_order,1),-np.array(getCoeff(k,n_order,0))]#error:bad operator - for list
            mtxA[2*len(start_pos)+6+(n-2)*6+k-1, (n-2)*(n_order+1):((n-1)*(n_order+1))] = getCoeff(k,n_order,1)#error:bad operator - for list
            mtxA[2*len(start_pos)+6+(n-2)*6+k-1, (n-1)*(n_order+1):(n*(n_order+1))]=-np.array(getCoeff(k,n_order,0))
    ############################for Bmtx########################################
    mtxb = np.zeros((1,(n_order+1)*len(start_pos)))
    for i in range(0,len(start_pos)):
        mtxb[0,i] = start_pos[i]
        mtxb[0,i+(len(start_pos))] = goal_pos[i]

    
    print(np.shape(mtxA),np.shape(mtxb))
    return mtxA,mtxb
# def getConstrainMtxb(start_pos,goal_pos,n_order):
#     """
#     return the constraint matrix, for single coordinate
#     Args:
#         start_pos (list): start point of each sgment dim(1,n)
#         goal_pos (list): goal point of each sgment dim(1,n)
#         n_order (int): order of polynomial

#     Returns:
#         np.array: the constraint matrix
#     """
    
#     return 0

#list inserting test
# a=[0,0,0,0]
# b=[1,1]
# a[1:3]=b
# # print(a)
# # print([0]*8)
# # print(getCoeff(1,7,1))
# # print(getCoeff(2,7,1))
# # print(getCoeff(3,7,1))
# c=np.array([0,0,0,0])
# c[1:3]=[1,1]
# #print(c[-3:])
# d1=[40,80,40,30]
# d2=[80,40,30,50]
# mtxA,mtxb=getConstrainMtx(d1,d2,7)
# parameters=np.linalg.pinv(mtxA)@mtxb.T#.inv singular
#print(parameters)
#print(np.linalg.solve(mtxA,mtxb.T))

# X=[40,80,120,80,40]
# x1=[40,80,120,80]
# x2=[80,120,80,40]
# Y=[40,80,40,0,40]
# y1=[40,80,40,0]
# y2=[80,40,0,40]


X=[40,80,120,150,180]
x1=[40,80,120,150]
x2=[80,120,150,180]
Y=[40,80,40,0,40]
y1=[40,80,40,0]
y2=[80,40,0,40]


segment=len(x1)

mtxAx,mtxbx=getConstrainMtx(x1,x2,7)
param_x=np.linalg.inv(mtxAx)@mtxbx.T#ERROR:singular,说明矩阵错误

mtxAy,mtxby=getConstrainMtx(y1,y2,7)
param_y=np.linalg.inv(mtxAy)@mtxby.T
# plt.plot(X, Y, color="blue", linewidth=2.5, linestyle="-")
# plt.show()
print(np.shape(param_y[0:8]))
midpointx=[]
midpointy=[]
step=0.1

for seg in range(0,segment):
    for i in np.arange(0,1+step,step):#ERROR:step 较小更好,注意+step
        midpointx.append(float(getCoeff(0,7,i)@param_x[seg*8:(seg+1)*8]))
        #print(i)
        midpointy.append(float(getCoeff(0,7,i)@param_y[seg*8:(seg+1)*8]))
        
plt.plot(midpointx, midpointy, color="blue", linewidth=2.5, linestyle="-")
plt.plot(X, Y)
plt.show()

# x1=[40,80]
# x2=[80,120]
# mtxAx,mtxbx=getConstrainMtx(x1,x2,7)
#进行debug,发现第4行全为0，即index出现问题
# print(np.shape(mtxAx))
# param_x=np.linalg.inv(mtxAx)@mtxbx.T