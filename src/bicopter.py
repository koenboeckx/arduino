import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

from .utilities import System, ADD_NOISE

class Bicopter(System):
    def __init__(self, T, add_noise=False):
        super().__init__(T, add_noise)
        self.params = {
            'V_min':        -10.0,              # minimim input voltage
            'V_max':         10.0,              # maximum input voltage
            'max_theta':     10/180 * np.pi,    # max allowed plane inclination = 10°
            'mass':          15,                # mass of beam
            'length':        1.2,               # length of beam
            'K':             5.0,               # motor coeff
        }
    
    def init_state(self):
        return np.array([0.0, 0.0]) # theta, omega
    
    def get_measurement(self):
        measurement = self.state[0]
        if self.add_noise:
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
