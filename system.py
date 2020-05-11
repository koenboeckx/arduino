"""System simulator
Usage: python3 system.py [bob|ship|maglev|bicopter]
"""

import threading, time, sys
from multiprocessing.connection import Listener

import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

ADD_NOISE = True

class System:
    def __init__(self, T):
        self.T = T # sampling period / step size
        self.t = 0 # time
        self.state = self.init_state()
        self.u = 0.0 # keep track of last given command
        self.params = {} # container for system parameters
        self.disturbance = 0.0 # use depends on application
    
    def get_state(self):
        return self.state[0], self.state[1]

    def set_u(self, u):
            self.u = self._constrain_input(u)
    
    def step(self):
        #self.state += self.T * self.deriv(self.t, self.state, self.u)  # Euler scheme
        self.state += self._rk4(self.t, self.state, self.u)             # Runge-Kutta 4
        self.t += self.T
        self._constrain_state()

    def _rk4(self, t, state, u):
        """Implements Runge-Kutta-4 integration scheme"""
        h = self.T
        k1 = h * self.deriv(t, state, u)
        k2 = h * self.deriv(t + h/2, state + k1/2, u)
        k3 = h * self.deriv(t + h/2, state + k2/2, u)
        k4 = h * self.deriv(t + h, state + k3, u)
        return 1/6*(k1 + 2*k2 +2*k3 + k4)

    def init_state(self):
        raise NotImplementedError
       
    def get_measurement(self):
        raise NotImplementedError

    def _constrain_input(self, u):
        raise NotImplementedError 
    
    def _constrain_state(self):
        raise NotImplementedError
    
    def deriv(self, t, x, u):
        """Computes first derivative"""
        raise NotImplementedError

class BallOnBeam(System):
    def __init__(self, T):
        super().__init__(T)
        self.params = {
            'L':            1.0,            # beam length from -L to L
            'radius':       0.1,            # radius of ball   
            'max_alpha':    10/180 * np.pi, # max allowed plane inclination = 10°
            'friction':     0.1,            # friction on ball
        }
    
    def init_state(self):
        return np.array([0.0, 0.0])
    
    def get_measurement(self):
        measurement = self.state[0] * np.cos(self.u)
        if ADD_NOISE:
            measurement += np.random.normal(scale=self.params['L']/50)
        return measurement
    
    def _constrain_input(self, u):
        max_alpha = self.params['max_alpha']
        if u < -max_alpha:
            return -max_alpha
        elif u > max_alpha:
            return max_alpha
        else:
            return u 
    
    def _constrain_state(self):
        x, v = self.state[0], self.state[1]
        L = self.params['L']
        if x < -L:
            x = -L
            v = 0.0
        elif x > L:
            x = L
            v = 0.0
        self.state[0] = x
        self.state[1] = v
    
    def deriv(self, t, x, u):
        """Computes first derivative"""
        x_dot = np.zeros((2, ))
        x_dot[0] = x[1]
        x_dot[1] = -5/7 * 9.81 * np.sin(u) - self.params["friction"]*x[1] + self.disturbance
        return x_dot

class MagLev(System):
    def __init__(self, T):
        super().__init__(T)
        self.params = {    
            'Z_min':        0.0,    # minimum position [cm]
            'Z_max':        10.0,   # maximum position [cm]
            'I_min':        0.0,    # minimum input current [A] 
            'I_max':        20.0,  # maximum input current [A]
            'imag_current': 3.0,    # current that represents static magnet [A]
            'mass':         0.5,    # mass of magnet [kg]
            'grav':         9.81,
        }
        self.disturbance = 0.0
    
    def init_state(self):
        return np.array([5.0, 0.0])
    
    def get_measurement(self):
        height = self.params['Z_max'] - self.params['Z_min']
        measurement = self.state[0]
        if ADD_NOISE:
            measurement += np.random.normal(scale=height/50)
        return measurement
    
    def _constrain_input(self, u):
        if u < self.params['I_min']:
            return self.params['I_min']
        elif u > self.params['I_max']:
            return self.params['I_max']
        else:
            return u 
    
    def _constrain_state(self):
        self.params['Z_min'] = self.disturbance
        x, v = self.state[0], self.state[1]
        if x < self.params['Z_min']:
            x = self.params['Z_min']
            v = 0.0
        elif x > self.params['Z_max']:
            x = self.params['Z_max']
            v = 0.0
        self.state[0] = x
        self.state[1] = v
    
    def _nonlinear(self, z):
        k0 = 0.1
        k1 = 1.262
        y = k0 + k1/(z + 2.)
        return y
    
    def deriv(self, t, x, u):
        """Computes first derivative"""
        Im = self.params['imag_current']
        mass = self.params['mass']
        grav = self.params['grav']

        z, v = x[0], x[1]

        y = self._nonlinear(z)

        x_dot = np.zeros((2, ))
        x_dot[0] = v
        x_dot[1] = 1/mass * (-y * (u + Im) + mass*grav)
        return x_dot

class Bicopter(System):
    def __init__(self, T):
        super().__init__(T)
        self.params = {
            'V_min':        -10.0,              # minimim input voltage
            'V_max':         10.0,              # maximum input voltage
            'max_theta':     10/180 * np.pi,    # max allowed plane inclination = 10°
            'mass':          15,                # mass of beam
            'length':        1.2,               # length of beam
            'K':             5.0,             # motor coeff
        }
    
    def init_state(self):
        return np.array([0.0, 0.0]) # theta, omega
    
    def get_measurement(self):
        measurement = self.state[0]
        if ADD_NOISE:
            measurement += np.random.normal(scale=0.01)
        return measurement
    
    def _constrain_input(self, u):
        if u < self.params['V_min']:
            return self.params['V_min']
        elif u > self.params['V_max']:
            return self.params['V_max']
        else:
            return u 
    
    def _constrain_state(self):
        theta, omega = self.state[0], self.state[1]
        if theta < -self.params['max_theta']:
            theta = -self.params['max_theta']
            omega = -0.1 * omega
        elif theta > self.params['max_theta']:
            theta = self.params['max_theta']
            omega = -0.1 * omega
        self.state[0] = theta
        self.state[1] = omega

    def deriv(self, t, x, u):
        """Computes first derivative"""
        mass   = self.params['mass']
        length = self.params['length']
        K      = self.params['K']
        _, omega = x[0], x[1]

        x_dot = np.zeros((2, ))
        x_dot[0] = omega
        x_dot[1] = -6./(mass*length) * K*u + self.disturbance
        return x_dot

class Ship(System):
    def __init__(self, T):
        super().__init__(T)
        self.params = {
            'K':         1.4067,    # K = g*d - G*h [Nm] - see accompanying document
            'l':         0.15,      # length of the arm [m]
            'g':         2.0,       # mass mounted at the end of the arm [kg]
            'C':         0.0619,    # damping coefficient of the water [N*s/m]
            'Ixx':       0.1748,    # moment of inertia around X-axis [kg*m^2]
            'phi_min':   -1.57,     # minimum input angle that can be applied
            'phi_max':    1.57,     # maximum input angle that can be applied
            'theta_min': -1.0,      # minimum rotation angle that can be applied
            'theta_max':  1.0,      # maximum rotation angle that can be applied
        }
    
    def init_state(self):
        return np.array([0.0, 0.0]) # theta, omega
    
    def get_measurement(self):
        measurement = self.state[0]
        if ADD_NOISE:
            measurement += np.random.normal(scale=0.01)
        return measurement
    
    def _constrain_input(self, u):
        if u < self.params['phi_min']:
            return self.params['phi_min']
        elif u > self.params['phi_max']:
            return self.params['phi_max']
        else:
            return u 
    
    def _constrain_state(self):
        theta, omega = self.state[0], self.state[1]
        theta += self.disturbance
        if theta < self.params['theta_min']:
            theta = self.params['theta_min']
            omega = 0.0
        elif theta > self.params['theta_max']:
            theta = self.params['theta_max']
            omega = 0.0
        self.state[0] = theta
        self.state[1] = omega

    def deriv(self, t, x, u):
        """Computes first derivative"""
        K   = self.params['K']
        g   = self.params['g']
        l   = self.params['l']
        C   = self.params['C']
        Ixx = self.params['Ixx']
        theta, omega = x[0], x[1]
        phi = self.u

        x_dot = np.zeros((2, ))
        x_dot[0] = omega 
        x_dot[1] = 1/Ixx * (-K*np.sin(theta) + g*l*np.sin(phi)*np.cos(theta) - C*omega) 
        return x_dot    
        
def counter():
    i = 0
    while True:
        yield i
        i += 1

class BoBGraph:
    """
    Shows the movement of the ball-on-beam
    """

    def __init__(self, system):
        self.system = system
        self.L = system.params['L']
        self.r = system.params['radius']
        self.sampling_period = system.T

        # Represent the QuadCopter
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-1.5*self.L, 1.5*self.L, -0.5, 0.5])

        self.update()
 
        self.text  = self.ax.text(-0.0, 0.31, '')
        self.beam, = self.ax.plot([self.beam_start[0], self.beam_end[0]],
                                  [self.beam_start[1], self.beam_end[1]], 'r-', lw=2)

        self.ball = self.ax.add_patch(
            patches.Circle(
                (0.0, 0.0),  # (x, y)
                radius= self.r
            )
        )

    def get_state(self):
        x, v  = self.system.get_state()
        alpha = self.system.u
        return x, v, alpha

    def update(self, i=0):
        x, v, alpha = self.get_state()
        
        self.beam_start  = np.hstack((-self.L*np.cos(alpha), -self.L*np.sin(alpha)))
        self.beam_end    = np.hstack(( self.L*np.cos(alpha),  self.L*np.sin(alpha)))
        self.ball_center = np.hstack(( x*np.cos(alpha),  x*np.sin(alpha) + self.r))
        return x, v, alpha

    def show(self, i):
        x, v, alpha = self.update(i)
        self.beam.set_data([self.beam_start[0], self.beam_end[0]],
                           [self.beam_start[1], self.beam_end[1]])
        self.ball.center = self.ball_center[0], self.ball_center[1]                           
        self.text.set_text(r'x = {:+.2f}, v = {:+.2f}, $\alpha$ = {:.2f}'.format(x, v, alpha))

        return self.beam, self.ball, self.text

class MagLevGraph:
    """
    Shows the movement of the magnetic levitation
    """

    def __init__(self, system):
        self.system = system
        self.sampling_period = system.T
        self.height = self.system.params['Z_max'] - self.system.params['Z_min']

        # Represent the magnetic ball
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-9.0, 9.0, 0.0, 11.])

        self.update()
 
        self.text  = self.ax.text(0.5, 8.5, '')
        self.ball = self.ax.add_patch(
            patches.Circle(
                (0.0, 0.0),  # (x, y)
                radius= .25
            )
        )

    def get_state(self):
        z, v = self.system.get_state()
        I = self.system.u
        return z, v, I

    def update(self, i=0):
        z, v, I = self.get_state()
        
        self.ball_center = np.array([0.0, self.height-z])
        return z, v, I

    def show(self, i):
        z, v, I = self.update(i)
        self.ball.center = self.ball_center[0], self.ball_center[1]
        self.text.set_text(r"z = {:+.2f}, v = {:+.2f}, I = {:.2f}".format(z, v, I))

        return self.ball, self.text

class BicopterGraph:
    """
    Shows the movement of the pendulum
    """

    def __init__(self, system):
        self.system = system
        self.sampling_period = system.T
        self.L = self.system.params['length']

        # Represent the QuadCopter
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-2, 2, -0.5, 0.5])

        self.update()
 
        self.text  = self.ax.text(-0.0, 0.31, '')
        self.beam, = self.ax.plot([self.beam_start[0], self.beam_end[0]],
                                  [self.beam_start[1], self.beam_end[1]], 'or-', lw=2)

        self.center = self.ax.add_patch(
            patches.Circle(
                (0.0, 0.0),  # (x, y)
                radius= .025
            )
        )

    def get_state(self):
        theta, omega = self.system.get_state()
        V = self.system.u
        torque = self.system.disturbance
        return theta, omega, V, torque

    def update(self, i=0):
        theta, omega, V, torque = self.get_state()
        
        self.beam_start = np.hstack((-self.L/2*np.cos(theta), -self.L/2*np.sin(theta)))
        self.beam_end   = np.hstack(( self.L/2*np.cos(theta),  self.L/2*np.sin(theta)))
        return theta, omega, V, torque

    def show(self, i):
        theta, omega, V, torque = self.update(i)
        self.beam.set_data([self.beam_start[0], self.beam_end[0]],
                           [self.beam_start[1], self.beam_end[1]])
        self.text.set_text(r'$\theta$ = {:+.2f}, $\omega$ = {:+.2f}, $V$ = {:.2f}, $T$ = {:.2f}'.format(theta, omega, V, torque))

        return self.beam, self.text

class ShipGraph:
    """
    Shows the movement of the ship
    """

    def __init__(self, params):
        self.system = system
        self.sampling_period = system.T
        self.L = 2.0

        # Represent the QuadCopter
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-2, 2, -0.5, 0.5])

        self.update()
 
        self.text  = self.ax.text(-0.0, 0.31, '')
        self.beam, = self.ax.plot([self.beam_start[0], self.beam_end[0]],
                                  [self.beam_start[1], self.beam_end[1]], 'or-', lw=2)

        self.center = self.ax.add_patch(
            patches.Circle(
                (0.0, 0.0),  # (x, y)
                radius= .025
            )
        )

    def get_state(self):
        theta, omega = self.system.get_state()
        phi = self.system.u
        return theta, omega, phi

    def update(self, i=0):
        theta, omega, phi = self.get_state()
        
        self.beam_start = np.hstack((-self.L/2*np.cos(theta), -self.L/2*np.sin(theta)))
        self.beam_end   = np.hstack(( self.L/2*np.cos(theta),  self.L/2*np.sin(theta)))
        return theta, omega, phi

    def show(self, i):
        theta, omega, phi = self.update(i)
        self.beam.set_data([self.beam_start[0], self.beam_end[0]],
                           [self.beam_start[1], self.beam_end[1]])
        self.text.set_text(r"$\theta$ = {:+.2f}, $\Omega$ = {:+.2f}, $\Phi$ = {:.2f}".format(theta, omega, phi))

        return self.beam, self.text

def stepper(system):
    ticker = threading.Event()
    while not ticker.wait(system.T):
        system.step()
        #print(system.get_state())

def controller(system):
    address = ('localhost', 6000)
    listener = Listener(address,
                        authkey=b'secret')
    conn = listener.accept()
    print('connection accepted from ', listener.last_accepted)
    while True:
        msg = conn.recv()
        # do something with msg
        if msg == 'measure':
            conn.send(system.get_measurement())
        elif msg == 'command':
            u = conn.recv()
            try:
                u = float(u)
                system.set_u(u)
            except:
                raise ValueError(f"Command {u} not allowed")
        elif msg == 'disturbance':
            disturbance = conn.recv()
            print(f'disturbance: {disturbance}')
            try:
                disturbance = float(disturbance)
                system.disturbance = disturbance
            except:
                raise ValueError(f"Disturbance {disturbance} not allowed")
        elif msg == 'reset':
            system.state = system.init_state()
            system.u = 0.0
        if msg == 'close':
            conn.close()
            break
    listener.close()

def visualizer(graph):
    ani = animation.FuncAnimation(graph.fig, graph.show,
                                  counter(), blit=True,
                                  interval=graph.sampling_period,  # interval between frames in milliseconds
                                  repeat=False)
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('Please provide the kind of system [bob, maglev, bicopter, ship]')
    system_type = sys.argv[1]
    print(f"System type: {system_type}")
    u = 0.0
    step_size = 0.01

    if system_type == 'bob':
        system = BallOnBeam(T=step_size)
        graph = BoBGraph(system)
    elif system_type == 'maglev':
        system = MagLev(T=step_size)
        graph = MagLevGraph(system)
    elif system_type == 'bicopter':
        system = Bicopter(T=step_size)
        graph = BicopterGraph(system)
    elif system_type == 'ship':
        system = Ship(T=step_size)
        graph = ShipGraph(system)        

    # create and start the stepper thread
    step_thread = threading.Thread(target=stepper, args=(system, ))
    step_thread.start()

    # create and start the communication thread
    comms_thread = threading.Thread(target=controller, args=(system, ))
    comms_thread.start()

    # start the visualization
    visualizer(graph)