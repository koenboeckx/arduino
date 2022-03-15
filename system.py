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
LOCALHOST = '127.0.0.1'

def counter():
    i = 0
    while True:
        yield i
        i += 1

def stepper(system):
    ticker = threading.Event()
    while not ticker.wait(system.T):
        system.step()
        #print(system.get_state())

def controller(system):
    address = (LOCALHOST, 6000)
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
    
    if len(sys.argv) > 2:
        if sys.argv[2] == 'no_noise':
            ADD_NOISE = False
        elif sys.argv[2] == 'noise':
            ADD_NOISE = True
    print(f"System type: {system_type} {'with' if ADD_NOISE else 'without'} noise")
    u = 0.0
    step_size = 0.01

    if system_type == 'bob':
        from src.ballonbeam import BallOnBeam, BoBGraph
        system = BallOnBeam(T=step_size)
        graph = BoBGraph(system)
    elif system_type == 'maglev':
        from src.maglev import MagLev2, MagLevGraph
        system = MagLev2(T=step_size)
        graph = MagLevGraph(system)
    elif system_type == 'bicopter':
        from src.bicopter import Bicopter, BicopterGraph
        system = Bicopter(T=step_size)
        graph = BicopterGraph(system)
    elif system_type == 'ship':
        from src.ship import Ship, ShipGraph
        system = Ship(T=step_size)
        graph = ShipGraph(system)
    elif system_type == '   ':
        from src.pendulum import Pendulum, PendulumGraph
        system = Pendulum(T=step_size, add_noise=ADD_NOISE)
        graph = PendulumGraph(system)
    else:
        raise ValueError(f"System '{system_type}' is unknown")


    # create and start the stepper thread
    step_thread = threading.Thread(target=stepper, args=(system, ))
    step_thread.start()

    # create and start the communication thread
    comms_thread = threading.Thread(target=controller, args=(system, ))
    comms_thread.start()

    # start the visualization
    visualizer(graph)
