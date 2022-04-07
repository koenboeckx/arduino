import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import matplotlib.patches as patches
import matplotlib.animation as animation

from .utilities import System, ADD_NOISE

class BallOnBeam(System):
    def __init__(self, T, add_noise=False):
        super().__init__(T, add_noise)
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
        if self.add_noise:
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
    pendulum = BallOnBeam(0.1)
    pg = BoBGraph(pendulum)
    visualizer(pg)