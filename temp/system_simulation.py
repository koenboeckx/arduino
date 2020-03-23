#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Nov 21 2017

@author: koen

A graphical simulator of an inverted pendulum.
To be used with (lqr) controller

System modelling based on equations from:
    http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling

Change: 27/11
    Corrected the ClassicalController class
    Added a System.set_check method, that can be used for measuring responses

To do : add observer => DONE 29/11/17
Changed: Made the use of variables more consistent (mostly column vectors) => DONE
"""

import numpy as np
from numpy import cos, sin, tan 
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import control
#from scipy import integrate

debug = False
print_state = True
LEFT, RIGHT  = -10, 10 # position of left and right boundary

#=====================================================================
#                           Mathematical Part
#=====================================================================


def step_response(sys, t, h):
    t0 = 5.0
    if np.abs(t - t0) < h:
        sys.set_reference(np.array([[0.0]]))

def disturbance(sys, t, h):
    """
    Disturbances introduced to system
    """
    if sys.__class__.__name__ == 'InvertedPendulum':
        if t % 5 < 2*h and t > 0.2: # hit pendulum every 5 seconds
            return np.array([0., 0., .0, 1.]).reshape((4, 1))
        else:
            return np.array([0., 0., 0., 0.]).reshape((4, 1))
    elif sys.__class__.__name__ == 'BallOnBeam':
        if t % 5 < h and t > 0.2: 
            choice = np.random.choice([-1, 1])
            return np.array([0., 0., 0., 0., choice*0.08]).reshape((5, 1))
        else:
            return np.array([0., 0., 0., 0., 0.]).reshape((5, 1))

class OptimalController:
    def __init__(self, sys, Q = None, R = None, limits=None):
        if Q is None:
            Q = np.eye(sys.n)
        if R is None:    
            R = 1
        
        self.sys = sys
        self.limits  = limits
        self.K, _, _ = control.lqr(sys.A, sys.B, Q, R)
        
        
        # This needs work: only works for w = 0
        
        self.Nu = np.linalg.pinv(-sys.C.dot((np.linalg.inv(sys.A - np.eye(sys.n))).dot(sys.B)))
        self.Nx = -np.linalg.inv((sys.A - np.eye(sys.n))).dot(sys.B).dot(self.Nu)
        
        print('K  =', self.K)
        print('Nu = ', self.Nu)
        print('Nx = ', self.Nx)
    
    def __call__(self, w, x):
        
#        m  = self.sys.n
#        x   = x.reshape((m,1))
#        u = self.Nu.dot(w) - self.K.dot(x - self.Nx.dot(w))
        
        u = w - np.dot(self.K, x)
        
        if self.limits is not None:
            if u < self.limits[0]:
                u = self.limits[0]
            elif u > self.limits[1]:
                u = self.limits[1]
        #return u.flatten()
        return u

class ClassicalController:
    """
    Classical Controller for the angle theta. Implement the recurrence equation
    in function __call__.
    It is practically to independently design a controller for theta that also 
    gives a stable result for x. To verify.
    """
    def __init__(self, sys, params):
        self.order = len(params['denominator']) - 1
        
        self.es = np.zeros(self.order+1)
        self.us = np.zeros(self.order+1)
        
        self.params_e = params['numerator']
        self.params_u = params['denominator']
        self.K        = params['K']
    def __call__(self,  w, x):
        e = w - x[3] # error on position x
        u  = np.sum([-self.us[i] * self.params_u[i+1] for i in range(self.order)])
        u += self.K*(self.params_e[0]*e + np.sum([self.es[i] * self.params_e[i+1] for i in range(self.order)]))
        for i in range(self.order, 0, -1):
            self.es[i] = self.es[i-1]
            self.us[i] = self.us[i-1]
        self.es[0] = e
        self.us[0] = u
        return u

class Observer:    
    def __init__(self, sys, h, Lambda, downsample=1):
        self.sys = sys
        self.h = h
        self.downsample = downsample
        sysc = control.ss(sys.A, sys.B, sys.C, np.zeros((2,1)))
        self.sysd = control.sample_system(sysc, h*downsample)
        self.L = control.place(self.sysd.A.T, self.sysd.C.T, Lambda)
        self.L = self.L.T
        print('L = ', self.L)
        self.x_est = sys.x0
        self.xs = np.zeros((sys.n, sys.samples+2))
        self.i = 0
    def __call__(self):
        if self.i % self.downsample == 0:
            sys = self.sys
            t,u,y = sys.get_state()
    
            # 1. Predict
            self.x_est  = self.sysd.A.dot(self.x_est) + self.sysd.B.dot(u.reshape((1,1)))
            # 2. Correct
            self.x_est += self.L.dot(y - self.sysd.C.dot(self.x_est))
        self.xs[:, self.i] = self.x_est.flatten()
        self.i += 1
        
        return self.x_est
    
    def get_results(self):
        return self.xs

class System:
    """
        General superclass to represent a system
    """
    def __init__(self, t0, x0, h, sytem_params, samples=100, noise_var = 0.0,
                 nonlinear=True, step='rk4', downsample=1):
        self.t0 = t0
        self.n = len(x0) # number of state variables
        self.x0 = x0.reshape((self.n, 1))
        self.i  = 0 # iteration
        self.t  = self.t0
        self.x  = self.x0
        self.samples = samples
        self.h = h
        
        self.params = sytem_params # define specific system params
        self.set_state_space_equations()
        _, n = np.shape(self.B)
        self.u = np.zeros((n, 1))
        
        self.noise_var = noise_var
        self.downsample = downsample
        
        if nonlinear: # set function that computes first derivative
            self.f = self._nonlinear
        else:
            self.f = self._linear       
        
        
        self.xs = np.zeros((self.n, self.samples+2))
        self.us = np.zeros(self.samples+2)
        self.ts = np.zeros(self.samples+2)
        
        self.reference = np.zeros((1, 1))
        self.regular_check = None
        self.controller = None
        self.observer = None
        self.step_type = step
        
    def step(self, x):
        """
            Function that moves the system forward
        """
        
        # Perform checks and (possibly) activate certain functions
        if self.controller: # should be moved to self.step; potential conflict with _nonlinear
            if self.i % self.downsample == 0: # only update u at certain time instances
                self.u = self.controller(self.reference, x)
        if self.regular_check is not None:
            self.regular_check(self, self.t, self.h)
        if self.observer is not None:
            self.observer()
        
        # Perform step forward, based on step_type
        if self.step_type == 'euler':
            self._euler()
        elif self.step_type == 'rk4':
            self._rk4()
    
    def set_observer(self, observer):
        self.observer = observer
    
    def __str__(self):
        return str('%s: t=%.2f, vars=%s' % (self.__class__.__name__, self.t, str(self.x)))
    
    def set_reference(self, w):
        self.reference = w
    
    def set_check(self, fn):
        """
            Function that will checked during each iteration; can be used to alter
            system variables. Commonly used to alter System.reference (through System.set_reference)
            in order to get step response
        """
        self.regular_check = fn
    
    def set_state_space_equations(self):
        """
            Based on system parameters, define the state-space equations 
            A, B, C and D. System-specific
        """
        raise NotImplementedError
    
    def _linear(self, t, x):
        """
            returns x'(t) = f(t, x(t)) with linear approximation
            Based on state-space equations, thus not longer system-dependent
        """

        x_dot  = np.dot(self.A, x) + np.dot(self.B, self.u)
               
        return x_dot
    
    def _nonlinear(self, t, x_):
        """
        Dynamic, non-linear system equations, based on system params.
        System-specific.
        """
        raise NotImplementedError
        
    def _euler(self):
        """
        Compute next step by simple Euler scheme.
        Additionally, add the linearities caused by physical constraints (walls, etc..)
        """

        self.x += self.h*self.f(self.t, self.x)
        
        # Add (pre-defined) disturbance
        self.x += disturbance(self, self.t, self.h)
        
        # Introduce noise to the state
        self.x += self.noise_var*np.random.randn(self.n, 1)
        self.t += self.h
        
        self._impose_boundaries()
        
        # Store values for plotting / analysis
        self.xs[:,self.i] = self.x.flatten()
        self.ts[self.i]   = self.t
        self.us[self.i]   = self.u
        self.i += 1
    
    def _rk4(self):
        """
        Compute next step with Runge-Kutta 4th order.
        Additionally, add the linearities caused by physical constraints (walls, etc..)
        """
        h, f = self.h, self.f
        x, t = self.x, self.t
        
        K0 = h*f(t, x)
        K1 = h*f(t + h/2, x + K0/2)
        K2 = h*f(t + h/2, x + K1/2)
        K3 = h*f(t + h, x + K2)
        self.x += 1/6*(K0+2*K1+2*K2+K3)
        
        # Add (pre-defined) disturbance
        self.x += disturbance(self, self.t, self.h)
        
        # Introduce noise to the state
        self.x += self.noise_var*np.random.randn(self.n, 1)
        
        self.t += h
        
        self._impose_boundaries()
        
        # Store values for plotting / analysis
        self.xs[:,self.i] = self.x.flatten()
        self.ts[self.i]   = self.t
        self.us[self.i]   = self.u
        self.i += 1
    
    def _impose_boundaries(self):
        """
        introduce system-specific boundary conditions (e.g. walls)
        """
        raise NotImplementedError
            
    def get_all_state_vars(self):
        return self.ts, self.xs, self.us
    
    def get_state(self):
        return self.t, self.u, np.dot(self.C, self.x)
    
    def update(self):
        self.step(self.x)
        return self.get_state()
    
    def set_controller(self, controller):
        self.controller = controller


class BallOnBeam(System):
    """
        Implementation of a ball rolling on a beam of which the angle is 
        controllable. This is done by means of a DC motor that positions
        the angle of the beam. Input is the voltage applied to the motor.
        Output is the position of the ball on the beam.    
    """
    def __init__(self, t0, x0, h, sytem_params, samples=100, noise_var = 0.0,
                 nonlinear=True, step='rk4', downsample=1):
        System.__init__(self, t0, x0, h, sytem_params, samples, noise_var,
                        nonlinear, step, downsample)
        self.reference = np.array([[0.0]])
        self.u = 0.0 # the bDC motor armature voltage Va
    
    def set_state_space_equations(self):
        m = self.params['m']
        r = self.params['r']
        l = self.params['l']
        g = self.params['g']
        R = self.params['R']
        L = self.params['L']
        Jr= self.params['Jr']
        psi = self.params['psi']
        b = self.params['b']
        
        self.l = l
        self.r = r
        
        Jb = 2/5*m*r**2
        
        # state variables = [current I, rotational speed omega,
        #                   rotation angle alpha, position x, velocity v]
        
        self.A = np.array([[-R/L,   -psi/L,     0.0,    0.0,    0.0],
                           [psi/Jr,    0.0,     0.0,    0.0,    0.0],
                           [   0.0,    1.0,     0.0,    0.0,    0.0],
                           [   0.0,    0.0,     0.0,    0.0,    1.0],
                           [   0.0,    0.0,  -1.0/(Jb/r**2 + m)*m*g,    0.0,    -b]])
#        self.B = np.array([[1/L,     0.0],
#                           [0.0,    -1/Jr],
#                           [0.0,     0.0],
#                           [0.0,     0.0],
#                           [0.0,     0.0]])
        self.B = np.array([[1/L],
                           [0.0],
                           [0.0],
                           [0.0],
                           [0.0]])
        self.C = np.array([[0.0, 0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0]])
        
    def _impose_boundaries(self):
        # Non-linearities
        # Ball hits end of beam
        if self.x[3] > self.l/2: 
            self.x[3] = self.l/2
            self.x[4] = -0.8*self.x[4]
        if self.x[3] < -self.l/2: 
            self.x[3] = -self.l/2
            self.x[4] = -0.8*self.x[4]
        
        # Beam inclinitation is limited
        if self.x[2] > 1.0: 
            self.x[2] = 1.0
        if self.x[2] < -1.0: 
            self.x[2] = -1.0
        
class InvertedPendulum(System):
    def __init__(self, t0, x0, h, sytem_params, samples=100, noise_var = 0.0,
                 nonlinear=True, step='rk4', downsample=1):
        System.__init__(self, t0, x0, h, sytem_params, samples, noise_var,
                        nonlinear, step, downsample)
        self.reference = np.array([[0.0]])
        
        self.alpha = 0.0 # angle of the beaam
    
    def set_state_space_equations(self):
        
        m = self.params['m']
        M = self.params['M']
        l = self.params['l']
        b = self.params['b']
        I = self.params['I']
        g = self.params['g']
        self.L = 2*l    # total length of pendulum
        
        # The linearized equations:
        # x' = Ax + Bu
        p = I*(m+M)+M*m*l**2;    # common denominator
        self.A = np.array([[0,  1,              0,              0],
                           [0,  -(I+m*l**2)*b/p, m**2*g*l**2/p,  0],
                           [0,  0,              0,              1],
                           [0,  -m*l*b/p,       m*g*l*(M+m)/p,  0]])
        
        self.B = np.array([[0],
                           [(I+m*l**2)/p],
                           [0],
                           [m*l/p]])
        self.C = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
  
    def _nonlinear(self, t, x_):
        """
            returns x'(t) = f(t, x(t)) with non-linear equations
        """
        m = self.params['m']
        M = self.params['M']
        l = self.params['l']
        b = self.params['b']
        I = self.params['I']
        g = self.params['g']
        x, v, theta, omega = x_
        
        if self.controller:
            if self.i % self.downsample == 0: # only update u at certain time instances
                self.u = self.controller(self.reference, x_)[0]
        else:
            self.u = np.array([0.0])

        omega_dot = (m*l*cos(theta))/((M+m)*(I+m*l**2) + (m*l*cos(theta))**2) * ((M+m)*g*tan(theta) - b*v - m*l*omega**2*sin(theta)+self.u)
        v_dot     = (I+m*l**2)/(m*l*cos(theta))*omega_dot - g*tan(theta)
        theta_dot = omega
        x_dot     = v
        
        x_dot = np.array([x_dot, v_dot, theta_dot, omega_dot]).reshape((4,1))

        return x_dot
    
    def _impose_boundaries(self):
        # Non-linearities
        # 1. Pendulum bounces back after hitting floor
        if self.x[2] > np.pi/2: 
            self.x[2] = np.pi/2
            self.x[3] = -.8*self.x[3] # velocity loss because of bounce
        if self.x[2] < -np.pi/2: 
            self.x[2] = -np.pi/2
            self.x[3] = -.8*self.x[3]
        # 2. Cart bouncees back after hitting walls
        if self.x[0] < LEFT:
            self.x[0] = LEFT
            self.x[1] = -.8*self.x[1]
        if self.x[0] > RIGHT:
            self.x[0] = RIGHT
            self.x[1] = -.8*self.x[1]
   
#=====================================================================
#                           Graphical Part
#=====================================================================
    
class InvertedPendulumGraph:
    """
        Wrapper around InvertedPendulum
        Shows the movement of the pendulum
    """
    def __init__(self, inverted_pendulum):
        self.ip = inverted_pendulum
        self.L = self.ip.L
        
        # Represent the IP
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        #self.ax.axis('equal')
        
        self.ax.axis([LEFT, RIGHT, -1.5*self.L, 3*self.L])
        self.cart_width  = 1.0
        self.cart_height = 0.2
                
        self.update()
        
        self.time_text = self.ax.text(LEFT+.5, 2.5*self.L, '')
        self.pendulum, = self.ax.plot([self.pendulum_start[0], self.pendulum_end[0]],
                                      [self.pendulum_start[1], self.pendulum_end[1]], 'or-', lw=2)
        self.cart = self.ax.add_patch(
            patches.Rectangle(
                         (self.x, 0.0) ,    # (x, y)
                         self.cart_width ,   # width
                         self.cart_height,  # height
                         )
            )
    
    def update(self, i=0):
        self.t, _, (self.x, self.theta) = self.ip.update()
        if print_state: print('%d: t=%.3f, x=%.2fm, theta=%.2frad' % (i, self.t, self.x, self.theta))
        self.pendulum_start = np.hstack((self.x +  self.cart_width/2, self.cart_height))
        self.pendulum_end   = self.pendulum_start + self.L*np.hstack((-np.sin(self.theta), np.cos(self.theta)))
    
    def show(self, i):
        self.update(i)
        self.pendulum.set_data([self.pendulum_start[0], self.pendulum_end[0]],
                               [self.pendulum_start[1], self.pendulum_end[1]])
        self.cart.set_x(self.x)
        text = 't = %.2fs, theta = %.2f°, x = %.2fm' % (self.t, 180*self.theta/np.pi, self.pendulum_end[0])
        self.time_text.set_text(text)
        return self.cart, self.pendulum, self.time_text
    
class BallOnBeamGraph:
    """
        Wrapper around BallOnBeam
        Shows the movement of the pendulum
    """
    def __init__(self, ballonbeam):
        self.bob = ballonbeam
        self.l = self.bob.l
        
        # Represent the IP
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')
        
        self.ax.axis([-self.bob.l/2, self.bob.l/2,
                      -self.bob.l/2, self.bob.l/2])
    
        self.alpha = self.bob.x[3]        
        self.factor = 10 # mutliplication factor to better visualize
        alpha = self.alpha * self.factor
                
        self.update()
        
        #self.time_text = self.ax.text(-self.bob.l/2, self.bob.l/2, '')
        self.time_text = self.ax.text(-.15, 0.1, '')
        
        along_x  = -self.bob.l/2*np.cos(alpha),  self.bob.l/2*np.cos(alpha)
        along_y =  -self.bob.l/2*np.sin(alpha),  self.bob.l/2*np.sin(alpha)
        self.beam, = self.ax.plot(along_x, along_y,'ob-', lw=2)
        
        pos = np.cos(alpha) * self.bob.x[3] - np.sin(alpha)*self.bob.r/2, np.sin(alpha) * self.bob.x[3] + np.cos(alpha)*self.bob.r/2        
        self.ball = self.ax.add_patch(
            patches.Circle(
                         pos ,    # (x, y)
                         self.bob.r/2,   # radius
                         color='r'
                         )
            )
    
    def update(self, i=0):
        self.t, _, s = self.bob.update()
        self.x = s[0]
        self.alpha = s[1]
        if print_state: print('%d: t=%.3f, x=%.2fmm, alpha=%.2frad ref=%f' % (i, self.t, 1e3*self.x, self.alpha, self.bob.reference))
    
    def show(self, i):
        self.update(i)
        
        alpha = self.alpha * self.factor
        along_x  = -self.bob.l/2*np.cos(alpha),  self.bob.l/2*np.cos(alpha)
        along_y =  -self.bob.l/2*np.sin(alpha),  self.bob.l/2*np.sin(alpha)
        self.beam.set_data(along_x, along_y)
        
        pos = np.cos(alpha) * self.x - np.sin(alpha)*self.bob.r/2, np.sin(alpha) * self.x + np.cos(alpha)*self.bob.r/2 
        self.ball.center = pos
        
        text = 't = %.2fs, theta = %.2f°, x = %.2fm' % (self.t, 180*self.alpha/np.pi, self.x)
        self.time_text.set_text(text)

        return self.beam, self.ball, self.time_text
        

### test functions
        
def test_ballonbeam():
    t0 = 0  
    x0 = np.array([.0, .0, .0, .0, .0]);
    h  = 0.005
    
    params = {
        # Set the values of the variables
        'm'  : .2,      # mass of the ball [kg]
        'r'  : .02,     # radius of ball [m]
        'l'  : .25,     # length of beam [m]
        'g'  : 9.81,    # acceleration due to gravity [m/s^2]
        'R'  : 3.0,     # resistance in electrical circuit [Ohm]
        'L'  : 1.0,     # inductance in electrical circuit [H]
        'psi': 2.0,     # flux of magnetic excitation [Wb]
        'Jr' : 2.0,     # inertia of the rotor shaft and beam [kg*m^2]
        'b'  : 0.0      # friction coeff
    }
    bob = BallOnBeam(t0, x0, h, sytem_params=params, samples=5000,
                              noise_var = 0.0001, nonlinear=False, step='rk4',
                              downsample=1)
    
    Q = np.array([[1, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0],
                  [0, 0, 1, 0, 0],
                  [0, 0, 0, 100, 0],
                  [0, 0, 0, 0, 1]])
    R = 1
    
    params = {'numerator'   : [0.0, 1.0000,   -3.8200,    5.4837,   -3.5060,    0.8423],
              'denominator' : [1.0000,   -3.8940,    6.0653,   -4.7237,    1.8394,   -0.2865],              
              'K'           : -408}

    controller =  OptimalController(bob, Q, R, limits=np.array([-1., 1.]))
    #controller = ClassicalController(bob, params)
    bob.set_controller(controller)
    
    Lambda = [.5, .6, .7, .8, .9]
    observer = Observer(bob, h, Lambda)
    bob.set_observer(observer)
    
    bob.set_check(fn = step_response)
    
    
    bobgraph = BallOnBeamGraph(bob)
    
    ani = animation.FuncAnimation(bobgraph.fig, bobgraph.show, 
                                  np.arange(1, bob.samples), blit=True,
                                  interval=h*1000, # interval between frames in milliseconds
                                  repeat=False);
    
    plt.show()
    
    t,x,u = bob.get_all_state_vars()
    
    x_est = observer.get_results()
    
    plt.figure();
    plt.subplot(5,1,1);
    plt.plot(t, x[3,:], '-');
    plt.plot(t, x_est[3,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel('x');
    plt.subplot(5,1,2);
    plt.plot(t, x[4,:], '-');
    plt.plot(t, x_est[4,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\dot x$');
    plt.subplot(5,1,3);
    plt.plot(t, x[2,:], '-');
    plt.plot(t, x_est[2,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\alpha$');
    plt.subplot(5,1,4);
    plt.plot(t, x[1,:], '-');
    plt.plot(t, x_est[1,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\omega$');
    plt.subplot(5,1,5);
    plt.plot(t, u,'-')
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$V_a$');
    plt.show()
        

def test_pendulum():
    t0 = 0  
    x0 = np.array([0., 0., .0, 0.]);
    h  = 0.01
    downsample_ratio = 10
    
    params = {
        # Set the values of the variables
        'M'  : .5,      # mass of the cart [kg]
        'm'  : .2,      # mass of the pendulum [kg]
        'b'  : 0.1,     # coefficient of friction for cart [N/m/sec]
        'l'  : 1.5,     # length to pendulum center of mass [m]
        'I'  : 0.006,   # mass moment of inertia of the pendulum [kg.m^2]
        'g'  : 9.81     # acceleration due to gravity [m/s^2]
    }
    
    ip = InvertedPendulum(t0, x0, h, sytem_params=params, samples=2500, 
                          noise_var = 0.005, nonlinear=True, 
                          step='rk4', downsample=downsample_ratio)
    
    # For optimal controller
    Q = np.array([[1, 0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 0]])
    R = 1
    
    # for classical controller
    params = {'denominator' : [1.0000,   -1.4966,    0.4966],
              'numerator'   : [1.0000,   -1.918,    0.9204],
              'K'           : 388}
    
    controller =  OptimalController(ip, Q, R)
    
    #controller = ClassicalController(ip, params)
    ip.set_controller(controller)
    
    Lambda = [.6, .65, .7, .75]
    Lambda = [.3, .35, .4, .45]
    observer = Observer(ip, h, Lambda, downsample=downsample_ratio)
    ip.set_observer(observer)
    
    #ip.set_check(fn = step_response)  

    ipgraph = InvertedPendulumGraph(ip)    
    ani = animation.FuncAnimation(ipgraph.fig, ipgraph.show, 
                                  np.arange(1, ip.samples), blit=True,
                                  interval=h*1000, # interval between frames in milliseconds
                                  repeat=False);
    plt.show()
    
    t,x,u = ip.get_all_state_vars()
    
    x_est = observer.get_results()
    
    plt.figure();
    plt.subplot(5,1,1);
    plt.plot(t, x[0,:], '-')
    plt.plot(t, x_est[0,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel('x');
    plt.subplot(5,1,2);
    plt.plot(t, x[1,:], '-')
    plt.plot(t, x_est[1,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\dot x$');
    plt.subplot(5,1,3)
    plt.plot(t, x[2,:], '-')
    plt.plot(t, x_est[2,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\theta$');
    plt.subplot(5,1,4);
    plt.plot(t, x[3,:], '-')
    plt.plot(t, x_est[3,:], 'r-', alpha=0.2, lw=5)
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$\omega$');
    plt.subplot(5,1,5);
    plt.plot(t, u, '-')
    plt.grid(True); plt.xlabel('t');plt.ylabel(r'$F$');
    plt.show()


    
if __name__ == '__main__':
     #test_ballonbeam()
     test_pendulum()
     
    