import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

from .utilities import System

DAMP_COEFF = 0 #0.5

class Pendulum(System):
    """ 
    https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
    """
    def __init__(self, T, add_noise=False):
        super().__init__(T)
        self.params = {
            'L':            0.3,            # pendulum length (m)
            'M':            0.5,            # mass of cart (kg)
            'm':            0.2,            # mass of pendulum (kg)
            'friction':     0.1,            # friction coefficient for cart
            'max_F':        10.,            # max allowed force (N)
            'rail_length':  1.,             # length of the rail
            'cart_length':  0.2,            # half length of cart
        }
        self.add_noise = add_noise
    
    def init_state(self):
        return np.array([0.0, 0.0, 0.01, 0.0]) # x0, v0, phi0, omega0
    
    def get_measurement(self):
        measurement = [self.state[0], self.state[2]]
        if self.add_noise:
            measurement[0] += np.random.normal(scale=0.01)
            measurement[1] += np.random.normal(scale=0.005)
        return measurement
    
    def _constrain_input(self, u):
        max_F = self.params['max_F']
        if u < -max_F:
            return -max_F
        elif u > max_F:
            return max_F
        else:
            return u 
    
    def _constrain_state(self):
        x, v = self.state[0], self.state[1]
        phi, omega = self.state[2], self.state[3]
        length = self.params['rail_length']
        if x < -length+self.params['cart_length']/2:
            x = -length+self.params['cart_length']/2
            v = -DAMP_COEFF*v # cart bounces back
        elif x > length-2*self.params['cart_length']/2:
            x = length-2*self.params['cart_length']/2
            v = -DAMP_COEFF*v
        if phi > np.pi/2.1:
            phi = np.pi/2.1
            omega = -DAMP_COEFF*omega
        elif phi < -np.pi/2.1:
            phi = -np.pi/2.1
            omega = -DAMP_COEFF*omega
        self.state[0] = x
        self.state[1] = v
        self.state[2] = phi
        self.state[3] = omega
    
    def deriv(self, t, x, u):
        """Computes first derivatives"""
        L = self.params['L']
        m = self.params['m']
        M = self.params['M']
        b = self.params['friction']
        #I = 1/3*m*L**2 # moment of inertia about the end of the rod
        I = 0.006
        g = 9.81

        N = I*(M+m) + M*m*L**2
        x_dot = np.zeros((4, ))
        x_dot[0] = x[1]
        x_dot[1] = (-(I+m*L**2)*b*x[1] + m**2*g*L**2 * x[2] + (I+m*L**2)*u)/N
        x_dot[2] = x[3]
        x_dot[3] = (-m*L*b * x[1] + m*g*L*(M+m) * x[2] + m*L*u)/N
        return x_dot
    
    def get_state(self):
        return self.state

class PendulumGraph:
    def __init__(self, system) -> None:
        self.system = system
        self.L = system.params['L']
        self.sampling_period = system.T
        
        # Represent the inverted pendulum
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('scaled')

        length = self.system.params['rail_length']
        self.ax.axis([-1.1*length, 1.1*length, -0.1,  2.5*self.L])
        self.cart_l = self.system.params['cart_length']

        self.update()

        self.text  = self.ax.text(-0.9*length, 2*self.L, '')
        self.cart = self.ax.add_patch(
            patches.Rectangle(
                (0.0-self.cart_l, 0.0),  # (x, y)
                width=2*self.cart_l, height=self.cart_l
            )
        )
        self.pendulum, = self.ax.plot([self.pendulum_start[0], self.pendulum_end[0]],
                                      [self.pendulum_start[1], self.pendulum_end[1]], 'r-', lw=3)
    

    def get_state(self):
        x, v, phi, omega  = self.system.get_state()
        F = self.system.u
        return x, v, phi, omega, F
    
    def update(self, i=0):
        x, v, phi, omega, F = self.get_state()
        
        self.pendulum_start  = np.hstack((x , 0.9*self.cart_l))
        self.pendulum_end    = np.hstack((x - self.L*np.sin(phi),
                                          0.9*self.cart_l + self.L*np.cos(phi)))
        return x, v, phi, omega, F

    def show(self, i):
        x, v, phi, omega, F = self.update(i)
        self.pendulum.set_data([self.pendulum_start[0], self.pendulum_end[0]],
                               [self.pendulum_start[1], self.pendulum_end[1]])
        self.cart.set_x(x-self.cart_l)                         
        #self.text.set_text(r'x = {:+.2f}, v = {:+.2f}, $\phi$ = {:.2f}, F = {:+.2f}'.format(x, v, phi, F))
        self.text.set_text(f'x = {x:+.2f}, v = {v:+.2f}, $\phi$ = {phi:.2f}, F = {F:+.2f}')

        return self.cart, self.pendulum, self.text


if __name__ == '__main__':
    import matplotlib.animation as animation
    def counter():
        i = 0
        while True:
            yield i
            i += 1

    def visualizer(graph):
        ani = animation.FuncAnimation(graph.fig, graph.show,
                                    counter(), blit=True,
                                    interval=graph.sampling_period,  # interval between frames in milliseconds
                                    repeat=False)
        plt.show()
    
    pendulum = Pendulum(0.1)
    pg = PendulumGraph(pendulum)
    visualizer(pg)