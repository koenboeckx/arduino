"""System simulator"""

# see: https://stackoverflow.com/questions/2291772/virtual-serial-device-in-python
# with pty


import threading, time
from multiprocessing.connection import Listener

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

ADD_NOISE = False

class BallOnBeam:
    def __init__(self, T):
        self.T = T # sampling period / step size
        self.t = 0 # time
        self.state = self.init_state()
        self.u = 0.0 # keep track of last given command
        self.params = {
            'L': 1.0,
            'radius': 0.1,
            'max_alpha': 10/180 * np.pi, # max allowed plane inclination = 10°
        }
    
    def init_state(self):
        return np.array([1.0, 1.0])
    
    def get_state(self):
        return self.state[0], self.state[1]
    
    def get_measurement(self):
        measurement = self.state[0]# * np.cos(self.u)
        if ADD_NOISE:
            measurement += np.random.normal(scale=self.params['L']/20)
        return measurement
    
    def set_u(self, u):
        self.u = self._constrain_input(u)
    
    def step(self):
        #self.state += self.T * self.deriv(self.t, self.state, self.u)
        self.state += self._rk4(self.t, self.state, self.u)
        #print(self.state)
        self.t += self.T
        self._constrain_state()

    def _rk4(self, t, state, u):
        h = self.T
        k1 = h * self.deriv(t, state, u)
        k2 = h * self.deriv(t + h/2, state + k1/2, u)
        k3 = h * self.deriv(t + h/2, state + k2/2, u)
        k4 = h * self.deriv(t, state + k3, u)
        return 1/6*(k1 + k2 + k3 + k4)
    
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
        x_dot[1] = -5/7 * 9.81 * np.sin(u)
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

    def __init__(self, bob):
        self.bob = bob
        self.L = bob.params['L']
        self.r = bob.params['radius']
        self.sampling_period = bob.T

        # Represent the QuadCopter
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-2, 2, -0.5, 0.5])

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
        x, v  = self.bob.get_state()
        alpha = self.bob.u
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
    u = 0.0
    system = BallOnBeam(T=0.001)

    # create and start the stepper thread
    step_thread = threading.Thread(target=stepper, args=(system, ))
    step_thread.start()

    # create and start the communication thread
    comms_thread = threading.Thread(target=controller, args=(system, ))
    comms_thread.start()

    # create and start the visualisation thread
    graph = BoBGraph(system)
    visualizer(graph)
