import serial

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import sys

params = {'bus_name': '/dev/ttyUSB0',
          'baudrate': 115200,
          'L': 1.0,
          'sampling_period': 10, # in milliseconds
          'vars':   ['z', 'v', 'I']
          }

class MagLevGraph:
    """
    Shows the movement of the magnetic levitation
    """

    def __init__(self, params):
        self.ser = serial.Serial(params['bus_name'], params['baudrate'])
        self.L = params['L']

        # Represent the magnetic ball
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False)
        self.ax.grid()
        self.ax.axis('equal')

        self.ax.axis([-7.0, 7.0, -1.0, 10.])

        self.update()
 
        self.text  = self.ax.text(0.5, 8.5, '')
        self.ball = self.ax.add_patch(
            patches.Circle(
                (0.0, 0.0),  # (x, y)
                radius= .25
            )
        )

    def get_state(self):
        while True:
            x = self.ser.readline()
            print(x)
            x = x.rsplit()
            if len(x) >= len(params['vars']): # avoids measuring theta too early
                break
        try:
            z = float(x[0])
            v = float(x[1])
            I = float(x[2])
        except:
            z, v, I = 0.0, 0.0, 0.0
        return z, v, I

    def update(self, i=0):
        z, v, I = self.get_state()
        
        self.ball_center = np.array([0.0, z])
        return z, v, I

    def show(self, i):
        z, v, I = self.update(i)
        self.ball.center = self.ball_center[0], self.ball_center[1]
        self.text.set_text(r"z = {:+.2f}, v = {:+.2f}, I = {:.2f}".format(z, v, I))

        return self.ball, self.text

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
    graph = MagLevGraph(params)
    print('Connected')
    ani = animation.FuncAnimation(graph.fig, graph.show,
                                  counter(), blit=True,
                                  interval=params['sampling_period'],  # interval between frames in milliseconds
                                  repeat=False)
    plt.show()

