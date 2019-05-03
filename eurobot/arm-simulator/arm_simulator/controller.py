import matplotlib.pylab as plt
import numpy as np
from numpy.linalg import norm
import math
from math import cos
from math import sin

def constrainAngle(x):
    x = math.fmod(x + np.pi,2*np.pi)
    if x < 0:
        x += 2*np.pi
    return x - np.pi

def angleDiff(a,b):
    dif = math.fmod(a - b + np.pi,2*np.pi);
    if dif < 0:
        dif += np.pi*2;
    return dif - np.pi;

class LQR:
    """
    Design a LQR regulator for discret time systems with finite horizon

    J = sum ( xQx + uRu + 2xNu )
    u = -Fx
    
    Dimensions:
        - n  : number of joints
        - p  : number of controlled state
    Parameters:
        - Ac : State matrix in continious time [n * n]
        - Bc : Input matrix in continious time [n * p]
        - Q  : State design matrix [n * n]
        - R  : Input design matrix [p * p]
        - N  : Mixed design matrix [n * p]
        - T  : Period of sampling
        - h  : Horizon
    """
    def __init__(self,Ac,Bc,Q,R,N,T,h):
        self.A, self.B = self.discretize(Ac,Bc,T)
        P = self.P_k(self.A,self.B,Q,R,N,h)
        self.F = self.F_k(self.A,self.B,P,R,N,h)
    
    """
    Compute F(k) the feedback gain matrix

    Return:
        - F : the optimal feedback gain matrix
    """
    def F_k(self,A,B,P,R,N,h):
        F = []
        B_T = np.transpose(B)
        N_T = np.transpose(N)
        
        for k in range(h):
            F.append( np.linalg.inv(R+B_T@P[k+1]@B) @ (B_T@P[k+1]@A + N_T) )
        return F

    """
    Compute P(k) iteratively backwards in time by dynamic riccati equation

    Dimension:
        - n : state size
    Return:
        - P : diagonal matrix [n * n]
    """
    def P_k(self,A,B,Q,R,N,h):
        
        P = []
        for i in range(h+1):
            P.append(np.zeros((A.shape[0],A.shape[1])))
        P[h] = Q
        
        A_T = np.transpose(A)
        B_T = np.transpose(B)
        N_T = np.transpose(N)
        
        if R.shape[0] > 1:
            for k in range(h,1,-1):
                P[k-1] = A_T@P[k]@A - (A_T@P[k]@B + N) @ np.linalg.inv(R+B_T@P[k]@B) @ (B_T@P[k]@A+N_T) + Q
        else:
            for k in range(h,1,-1):
                P[k-1] = A_T@P[k]@A - (A_T@P[k]@B + N) * np.linalg.inv(R+B_T@P[k]@B) @ (B_T@P[k]@A+N_T) + Q
        return P

    """
    Discretize matrice with a Tustin transform (linear approximation)
    for system x_k+1 = A x_k + B u_k
    
    A_d = e^(AT) ~= (I + 1/2AT)(I - 1/2AT)^(-1)
    B_d = A^(-1)(A_d - I)B
    
    where A and B are continious time matrix and A_d, B_d the discretized matrix with 
    controller frequency 1/T
    """
    def discretize(self,A,B,T):
        I = np.identity(A.shape[0])
        A_d = (I + 1/2*A*T)@np.linalg.inv(I-1/2*A*T)
        B_d = T*B
        return A_d,B_d

class trajectory:
    """
    A class to set a reference trajectory

    The trajectory is computed in the cartesian world. A straight and smooth movement toward 
    the target position is design to minimize the effort on the end joint
    
    Dimensions: 
        - m : cartesian coordinates
        
    Parameter:
        - r : the vector position of the target position [m * 1]
        - a : acceleration max
    
    """
    def __init__(self,r,a):
        self.dr = r
        self.t0 = math.sqrt(0.25*np.pi*np.linalg.norm(r)/a)
        self.v_max = 0.5 *np.linalg.norm(r) / self.t0

    """
    Return the instantaneous norm of velocity at time t 
    """
    def _norm_v(self,t):
        if t < 0:
            return 0
        elif t <= self.t0:
            return self.v_max/2 * np.sin(np.pi/self.t0*(t-self.t0/2)) + self.v_max/2
        elif t < 2*self.t0:
            return self.v_max
        elif t <= 3*self.t0:
            return self.v_max/2 * np.sin(np.pi/self.t0*(t-3/2*self.t0)) + self.v_max/2
        else:
            return 0
    
    """
    Return the distance traveled at time t 
    """
    def _norm_r(self,t):
        if t < 0:
            return 0
        elif t <= self.t0:
            return self.v_max/2*(-np.cos(np.pi/self.t0*(t-self.t0/2)))*self.t0/np.pi + self.v_max/2*t
        elif t < 2*self.t0:
            return self.v_max/2*self.t0 + self.v_max*(t-self.t0)
        elif t <= 3*self.t0:
            return self.v_max/2*self.t0 + self.v_max*(self.t0) + self.v_max/2*(-np.cos(np.pi/self.t0*(t+self.t0/2)))*self.t0/np.pi + self.v_max/2*(t-2*self.t0)
        else:
            return np.linalg.norm(self.dr)
    
    """
    Return the instantaneous norm of acceleration at time t 
    """
    def _norm_a(self,t):
        if t < 0:
            return 0
        elif t <= self.t0:
            return self.v_max/2*(np.cos(np.pi/self.t0*(t-self.t0/2)))*np.pi/self.t0
        elif t < 2*self.t0:
            return 0
        elif t <= 3*self.t0:
            return self.v_max/2*(np.cos(np.pi/self.t0*(t-3/2*self.t0)))*np.pi/self.t0
        else:
            return 0
    
    """
    Return the velocity vector in cartesian coordinates at time t
    """
    def v(self,t):
        return self.dr/np.linalg.norm(self.dr)*self._norm_v(t)
    
    """
    Return the position vector of the end joint in cartesian coordinates at time t
    """
    def r(self,t):
        return self.dr/np.linalg.norm(self.dr)*self._norm_r(t)
    
    """
    Return the acceleration vector in cartesian coordinates at time t
    """
    def a(self,t):
        return self.dr/np.linalg.norm(self.dr)*self._norm_a(t)
    
    """
    Return a reference vector in cartesian coordinates
    that contains position, velocity and acceleration at time t
    """
    def ref(self):
        return lambda self,t: np.array([self.r(t),self.v(t),self.a(t)])
    
    """
    Plot the norm of position, velocity and acceleration in function of time
    """
    def plot_norm(self):
        t = np.linspace(0, 3*self.t0)
        v = np.zeros(t.size)
        r = np.zeros(t.size)
        a = np.zeros(t.size)
        for i in range(t.size):
            v[i] = self._norm_v(t[i])
            r[i] = self._norm_r(t[i])
            a[i] = self._norm_a(t[i])
        plt.figure(1)
        line1, = plt.plot(t, r, '--', linewidth=2, label='r(t)')
        plt.legend(loc='lower left')
        plt.xlabel('time')
        plt.ylabel('|r|')
        plt.figure(2)
        line2, = plt.plot(t, v, '--', linewidth=2, label='v(t)')
        plt.legend(loc='lower left')
        plt.xlabel('time')
        plt.ylabel('|v|')
        plt.figure(3)
        line3, = plt.plot(t, a, '--', linewidth=2, label='a(t)')
        plt.legend(loc='lower left')
        plt.xlabel('time')
        plt.ylabel('|a|')
    
    """
    Plot the position y in function of time in x
    """
    def plot_ref(self):
        t = np.linspace(0, 3*self.t0)
        r = np.zeros((t.size,2))
        for i in range(t.size):
            r[i] = self.r(t[i])
        plt.figure(4)
        line1, = plt.plot(r[:,0], r[:,1], '*', linewidth=2, label='r(t)')
        plt.legend(loc='lower right')
        plt.xlabel('x')
        plt.ylabel('y')
        
class Arm2D:
    """
    Controller for a multi joints system in a 2D plan

    Dimensions:
        - n   : number of joints

    Parameters:
        - l   : length of consecutive arm segments from base to end [n]
    """
    def __init__(self, l,lqr):
        self.l   = np.array(l)
        self.n   = l.size
        self.lqr = lqr

    """
    return the position of the end joint
    """
    def getArmPos(self,q):
        p = np.array([[0.,0.]])
        R = lambda q: np.array([sin(q), cos(q)])
        _q = np.array(q)

        for i in range(self.n):
            if i > 0:
                _q[i] = constrainAngle(_q[i]+_q[i-1])
            p = np.concatenate((p,[p[i]+self.l[i]*R(_q[i])]), axis=0)
            #print(_q)
            #print(p)
        return p[-1]

    """
    Set new target
    """
    def setNewTarget(self,r,a,q):
        self.p0 = self.getArmPos(q)
        self.ref = trajectory(r-self.p0,a)
        self.t = 0


    def expected_derivate_state(self,q,t,dt):
        diff = np.array([0.,0.,0.])
        if t<0.:
            return np.array([0.,0.,0.])
        elif t>self.ref.t0*3:
            return np.array([0.,0.,0.])
        else:
            Target_k_2 = self.p0+self.ref.r(t+dt)
            Target_k_1 = self.p0+self.ref.r(t)
            #print('q',q)
            q_k_1 = self.step(q,Target_k_1)
            q_k_2 = self.step(q_k_1,Target_k_2)
            #print(q_k_1,q_k_2)

            for i in range(self.n):
                diff[i] = angleDiff(q_k_2[i],q_k_1[i])
                #print(lastTarget[i], newTarget[i])
                #print(i,angleDiff(lastTarget[i],newTarget[i]))
            #print('diff ',diff/dt)
            return diff/dt


    def simulate_sys(self,q_aug,dt):
        sim_len = 200
        _q_ = np.array([0.0,0.0,0.0])
        u = []
        state_k = np.array(q_aug)
        for j in range(sim_len):
            #print('k', state_k)
            state_k_1 = self.run(state_k,dt)
            #print('k', state_k)
            #print('k+1', state_k_1)
            u = - self.lqr.F[0]@state_k_1
            state_after_ctrl = self.lqr.A@state_k + self.lqr.B@u
            for i in range(3):
                _q_[i] = np.array(state_after_ctrl[2*i])
            #print('ctrl', state_after_ctrl)
            #print(u)
            print(self.getArmPos(_q_),self.p0+self.ref.r(self.t))
            #print(self.getArmPos(_q_))
            state_k = np.array(state_after_ctrl)

    def run(self,q_aug,dt):
        a_1 = []
        v_1 = []
        b = np.array([0.,0.])
        q_1 = np.array([0.0,0.0,0.0])
        self.t += dt
        q_out = np.array(q_aug)

        #print(system.getArmPos(b,q_1))
        _tar = self.p0+self.ref.r(self.t)

        for i in range(3):
            q_1[i] = np.array(q_aug[3*i])
        #print(self.getArmPos(b,q),self.ref.r(self.t))
        #print(_tar)
        #print(self.getArmPos(b,q_1))
        v_1 = self.expected_derivate_state(q_1,self.t,dt)
        q_1 = self.step(q_1,_tar)
        for i in range(self.n):
            #v_1.append(angleDiff(q_1[i],self.lastq[i])/dt)
            #a_1.append(angleDiff(v_1[-1],self.lastdq[i])/dt)
            q_out[2*i] = angleDiff(q_aug[2*i],np.array(q_1[i]))
            q_out[2*i+1]= angleDiff(q_aug[2*i+1],v_1[i])
            #q_out[3*i+2]= angleDiff(q_aug[3*i+2],a_1[i])
        #print(system.getArmPos(b,q_1))
        #print(self.lastdq)
        #print(e,de,dde)
        #print(q_1)
        #print(self.getArmPos(b,q_1))
        #return np.array(e), np.array(de), np.array(dde)
        #print(q_out)
        return np.array(q_out)

    """
    Perform an iteration:

    Dimensions:
        - m  : cartesian coordonates
        - n  : number of joints

    Parameters:
        - q  : joint angles [n]
        - b  : base world position [m]
        - tar: target world position [m]
    Return:
        - q  : joint angles [n]
    """
    def step(self,q, tar):
        #print(tar)
        b = np.array([0.,0.])
        p = np.array([b])
        R = lambda q: np.array([sin(q), cos(q)])
        _q = np.array(q)

        for i in range(self.n):
            if i > 0:
                _q[i] = constrainAngle(_q[i]+_q[i-1])
            p = np.concatenate((p,[p[i]+self.l[i]*R(_q[i])]), axis=0)
        q = self.inv_kinematic(p,tar)
        #print(p)
        #print(np.linalg.norm(p[3]-p[2]))
        #print(30*self.ref.t0)
        #print(q)
        return np.array(q)


    """
    Fast pseudo inverse kinematic for a multi joints arm
    
    q = phi⁻¹(x)

    Dimensions:
        - m  : cartesian coordonates
        - n  : number of joints

    Parameters:
        - p  : joint positions [n+1 * m]
        - tar: target position [m * 1]
        - tol: convergence tolerance

    Return:
        - q  : joint angles [n * 1]
    """
    def inv_kinematic(self, p, tar, tol=0.001):
        b = np.array([p[0]])
        r = np.zeros([self.n])
        s = np.zeros([self.n])
        q = np.zeros([self.n])
        if norm(tar) > sum(self.l):
            for i in range(self.n):
                r[i] = norm(p[i]-tar)
                s[i] = self.l[i]/r[i]
                p[i+1] = (1-s[i])*p[i] + s[i]*tar
        else:
            dif = norm(p[self.n]-tar)
            while dif > tol:
                p[self.n] = tar
                for i in range(self.n-1,-1,-1):
                    r[i] = norm(p[i+1] - p[i])
                    s[i] = self.l[i]/r[i]
                    p[i] = (1-s[i])*p[i+1]+s[i]*p[i]
                p[0] = np.array(b)
                for i in range(self.n):
                    r[i] = norm(p[i+1] - p[i])
                    s[i] = self.l[i]/r[i]
                    p[i+1] = (1 - s[i]) * p[i] + s[i]*p[i+1]
                dif = norm(p[self.n]-tar)
        #print(p[-1])
        #print(p)
        #pp = np.array(b)
        for i in range(self.n):
            #print(p[i+1]-p[i])
            q[i] = np.arctan2(p[i+1,0]-p[i,0],p[i+1,1]-p[i,1])
            ### some test
            #R = lambda q: np.array([sin(q), cos(q)])
            #pp = np.concatenate((pp,[pp[i]+self.l[i]*R(q[i])]), axis=0)
        qq = np.array(q)
        for i in range(self.n):
            if(i>0):
                q[i] = angleDiff(q[i],qq[i-1])
        #print(pp[-1])
        #print(self.getArmPos(b,q))
        return q
    
np.set_printoptions(precision=9)

#system test
de = 1e-9
#system.simulate_sys(q_aug_2,dt)
##################################
##################################
##################################
# testing arm with only 1 joint
REDUCTOR = 71

de = 1e-9
_A_ = np.array([
    [de,1.],
    [0.,de],
    ])

_B_ = REDUCTOR*np.array([
    [0.,],
    [1.,],
    ])

_Q_ = 10.*np.eye(2)
_R_ = 0.3*np.array([1])
_N_ = np.array(_B_)
T = 0.1
h = 10

lqr_1axe = LQR(_A_,_B_,_Q_,_R_,_N_,T,h)

target_1 = np.array([0.4,0.])
target_2 = np.array([0.0,0.])

state_1_k = np.array([0.0,0.])
state_2_k = np.array([0.0,0.])

masse_arm = 0.03
l = 0.06
I = masse_arm * l*l / 3.0

masse = 0.1
g = 9.81

ref2 = trajectory(target_1,2)
t = 0.

for i in range(20):
    #LQR fix ref
    u = - lqr_1axe.B@lqr_1axe.F[0]@(state_1_k - target_1 )
    #LQR dynamic ref
    target_2[0] = ref2.r(t)[0]
    target_2[1] = ref2.v(t)[0]
    u2 = - lqr_1axe.B@lqr_1axe.F[0]@(state_2_k - target_2 )

    state_1_k_1 = lqr_1axe.A@state_1_k + u
    state_2_k_1 = lqr_1axe.A@state_2_k + u2
    state_1_k = np.array(state_1_k_1)
    state_2_k = np.array(state_2_k_1)
    #print(u[1]*I+l*masse*g*np.sin(state_1_k[0]), state_1_k_1[0])
    print(u[1],u2[1])
    #print(state_2_k, ref2.r(t)[0],ref2.v(t)[0])
    #print(ref2.r(t)[0],ref2.v(t)[0])
    t = t+T