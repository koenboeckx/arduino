import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

from .utilities import System, ADD_NOISE



class Ship(System):
    def __init__(self, T, add_noise=False):
        super().__init__(T, add_noise)
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
        if self.add_noise:
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


class ShipGraph:
    """
    Shows the movement of the ship
    """

    def __init__(self, system):
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