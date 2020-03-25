from multiprocessing.connection import Client
import socket
import threading, time, sys, struct

import numpy as np

modes = {
    'OPEN_LOOP':    0,
    'CLASSICAL':    1,
    'STATE-SPACE':  2,
    'PROPORTIONAL': 3,
}

#### ------ only change Controller class ----------- ####

class Controller:
    "Your Controller"
    def __init__(self):
        self.pole = 0.85
        self.zero = 0.95
        self.k = 8.0

        self.u_prev = 0.0
        self.prev_error = 0.0
    
    def set_params(self, params):
        "params from matlab set_mode_params"
        self.zero = params[0]
        self.pole = params[1]
        self.k    = params[2]

    def __call__(self, y):
        error = params.w - y
        if params.mode == 'OPEN_LOOP':
            u = params.w
        elif params.mode == 'PROPORTIONAL':
            u = self.k*(params.w - y)
        elif params.mode == 'CLASSICAL':
            e1 = error
            u1 = -e1
            e2 = params.w - u1
            u2 = self.pole * self.u_prev + self.k*(e2 - self.zero*self.prev_error)
            self.u_prev = u2
            self.prev_error = e2
            u = u2
        elif params.mode == 'LL':
            u = self.pole * self.u_prev + self.k*(error - self.zero*self.prev_error)
            self.u_prev = u
            self.prev_error = error
        else:
            u = 0.0
        #print(f"y = {y:5.3f}, e = {error:5.3f}, u = {u:5.3f}")
        return u

#### ------ don't change anything below ----------- ####

def get_measurement(conn):
    conn.send('measure')
    y = conn.recv()
    return y

def set_u(conn, u):
    conn.send('command')
    conn.send(u)

def set_disturbance(conn, disturbance):
    conn.send('disturbance')
    conn.send(disturbance)

def system_comms(controller, params):
    address = ('localhost', 6000)
    with Client(address, authkey=b'secret') as conn:
        params.system_conn = conn
        while True:
            y = get_measurement(conn)
            u = controller(y)
            set_u(conn, u)

            # handle ongoing measurement
            if params.n_samples > 0:
                params.Y.append(y)
                params.U.append(np.float64(u))
                params.n_samples -= 1
                # when all needed samples are collected, send Y and U to matlab 
                if len(params.Y) > 0 and params.n_samples == 0: 
                    for item in params.Y:
                        params.matlab_conn.send(item)
                    for item in params.U:
                        params.matlab_conn.send(item)
                    params.Y = []
                    params.U = []

            time.sleep(params.Ts)

def read_float(conn):
    w_bytes  = conn.recv(4)
    try:
        w = struct.unpack('f', w_bytes)[0]
    except:
        print(w_bytes)
        print('ValueError!!!')
        w = 0.0
    return w

def read_int(conn):
    val_bytes = conn.recv(4)
    val = int.from_bytes(val_bytes, byteorder='little')
    return val

def matlab_comms(controller, params):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('localhost', params.ip)
    sock.bind(server_address)
    sock.listen(1)

    while True:
        # Wait for a connection
        connection, client_address = sock.accept()
        params.matlab_conn = connection
        print(connection, client_address)
        while True:
            data = connection.recv(4) # receive mode integer
            if data:
                mode = int.from_bytes(data, byteorder='little')
                print(f'mode = {mode}')
                if mode == 252: # set disturbance
                    disturbance = read_float(connection)
                    set_disturbance(params.system_conn, disturbance)
                elif mode == 253: # reset system
                    params.system_conn.send('reset')
                elif mode == 254: # close connection
                    print(f'Closing connection {connection}')
                    connection.close()
                elif mode == 255: # get response
                    params.w = read_float(connection)
                    params.n_samples = read_int(connection)
                    print(f'n_samples = {params.n_samples}')
                else:
                    params.w = read_float(connection)
                    n_params = read_int(connection)
                    if n_params > 0:
                        parameters = []
                        for _ in range(n_params):
                            parameters.append(read_float(connection))
                        controller.set_params(parameters)
                    if mode ==  modes['OPEN_LOOP']:
                        params.mode = 'OPEN_LOOP'
                    elif mode ==  modes['CLASSICAL']:
                        params.mode = 'CLASSICAL'
        connection.close()

class Params: # global object for comms between threads
    n_samples = 0
    Ts = 0.001
    ip = 6007
    mode = 'OPEN_LOOP'
    
    Y = []
    U = []
    matlab_conn = None
    system_conn = None
    w = 0.0
    reset = False
    

if __name__ == '__main__': 
    params = Params()
    if len(sys.argv) > 1:
        params.ip = int(sys.argv[1])
    if len(sys.argv) > 2:
        params.Ts = float(sys.argv[2])
    
    print(f'Sampling period = {params.Ts}\nListening on port {params.ip}')
    controller = Controller()

    sys_comms_thread = threading.Thread(target=system_comms, args=(controller, params))
    sys_comms_thread.start()

    matlab_comms_thread = threading.Thread(target=matlab_comms, args=(controller, params))
    matlab_comms_thread.start()
