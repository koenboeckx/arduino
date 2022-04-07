import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

from .utilities import System, ADD_NOISE

class MagLev(System):
    def __init__(self, T, add_noise=False):
        super().__init__(T, add_noise)
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
        if self.add_noise:
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

class MagLev2(MagLev):
    def __init__(self, T):
        super().__init__(T)
        self.params = {    
            'Z_min':        0.0,    # minimum position [cm]
            'Z_max':        10.0,   # maximum position [cm]
            'I_min':        0.0,    # minimum input current [A] 
            'I_max':        30.0,  # maximum input current [A]
            'imag_current': 0.0,    # current that represents static magnet [A]
            'mass':         0.5,    # mass of magnet [kg]
            'grav':         9.81,
        }
        self.disturbance = 0.0
    
    def _nonlinear(self, z):
        y =  2.5/(z + 1.0)
        return y



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