from serial import Serial

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import sys

params = {'bus_name': '/dev/ttyUSB0',
          'baudrate': 115200,
          'L': 0.3,
          'radius': .025,
          'sampling_period': 10, # in milliseconds
          'vars': ['x', 'v', 'alpha']
          }

class BoBGraph:
    """
    Shows the movement of the ball-on-beam
    """

    def __init__(self, params):
        self.ser = Serial(params['bus_name'], params['baudrate'])
        self.L = params['L']
        self.r = params['radius']

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
        while True:
            input = self.ser.readline()
            print(input)
            input = input.rsplit()
            if len(input) >= len(params['vars']): # avoids measuring theta too early
                break
        try:
            x     = float(input[0])
            v     = float(input[1])
            alpha = float(input[2])
        except:
            x, v, alpha  = 0.0, 0.0, 0.0
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

def counter():
    i = 0
    while True:
        yield i
        i += 1

def plot_history(system):
    thetas =  []
    for elem in system.history:
        thetas.append(elem[0])

    ts = np.arange(len(thetas))*system.h

    plt.figure(); plt.plot(ts, thetas)
    plt.xlabel('t [s]'); plt.ylabel(r'$\theta$ [rad]')
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        params['bus_name'] = sys.argv[1]
    if len(sys.argv) > 2:
        params['baudrate'] = int(sys.argv[2])
    print('Trying to connect on bus {} @ {} baud/s ...'.format(params['bus_name'],params['baudrate']))
    qcgraph = BoBGraph(params)
    print('Connected')
    ani = animation.FuncAnimation(qcgraph.fig, qcgraph.show,
                                  counter(), blit=True,
                                  interval=params['sampling_period'],  # interval between frames in milliseconds
                                  repeat=False)
    plt.show()

