from serial import Serial

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import sys

params = {'bus_name': '/dev/ttyUSB0',
          'baudrate': 115200,
          'L': 1.0,
          'sampling_period': 10, # in milliseconds
          }

class QuadcopterGraph:
    """
        Shows the movement of the pendulum
    """

    def __init__(self, params):
        self.ser = Serial(params['bus_name'], params['baudrate'])
        self.L = params['L']

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

    def get_theta(self):
        while True:
            x = self.ser.readline()
            print(x)
            x = x.rsplit()
            if len(x) > 4: # avoids measuring theta too early
                break
        try:
            theta = float(x[0])
            omega = float(x[1])
            ul    = float(x[2])
            ur    = float(x[3])
            T     = float(x[4])
        except:
            theta = 0.0
            omega = 0.0
            ul    = 0.0
            ur    = 0.0
            T     = 0.0
        return theta, omega, ul, ur, T

    def update(self, i=0):
        theta, omega, ul, ur, T = self.get_theta()
        
        self.beam_start = np.hstack((-self.L/2*np.cos(theta), -self.L/2*np.sin(theta)))
        self.beam_end   = np.hstack(( self.L/2*np.cos(theta),  self.L/2*np.sin(theta)))
        return theta, omega, ul, ur, T

    def show(self, i):
        theta, omega, ul, ur, T = self.update(i)
        self.beam.set_data([self.beam_start[0], self.beam_end[0]],
                           [self.beam_start[1], self.beam_end[1]])
        self.text.set_text(r'$\theta$ = {:+.2f}, $\omega$ = {:+.2f}, $u_l$ = {:.2f}, $u_r$ = {:.2f}, $T$ = {:.2f}'.format(theta, omega, ul, ur, T))

        return self.beam, self.text

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
    qcgraph = QuadcopterGraph(params)
    print('Connected')
    ani = animation.FuncAnimation(qcgraph.fig, qcgraph.show,
                                  counter(), blit=True,
                                  interval=params['sampling_period'],  # interval between frames in milliseconds
                                  repeat=False)
    plt.show()

