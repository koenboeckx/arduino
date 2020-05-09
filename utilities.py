from multiprocessing.connection import Client
import socket
import threading, time, sys, struct
import numpy as np

modes = {
    'OPEN_LOOP':    0,
    'CLASSICAL':    1,
    'STATE_SPACE':  2,
    'EXTENDED':     3,
    'TEST':         4,
}

class Params: # global object for comms between threads
    n_samples = 0
    Ts = 0.05
    ip = 6012
    mode = 'OPEN_LOOP'
    
    buffer = []
    matlab_conn = None
    system_conn = None
    w = 0.0
    reset = False

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
            u, results = controller(y)
            set_u(conn, u)

            # handle ongoing measurement
            if params.n_samples > 0:
                params.buffer.append(results)
                params.n_samples -= 1
                # when all needed samples are collected, send Y and U to matlab 
                if len(params.buffer) > 0 and params.n_samples == 0: 
                    N = np.float64(len(params.buffer[0]))
                    params.matlab_conn.send(N) # number of items per sample that are send back
                    for batch in params.buffer:
                        for item in batch:
                            params.matlab_conn.send(np.float64(item))
                    params.buffer = []
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
                    break
                elif mode == 255: # get response
                    params.w = read_float(connection)
                    params.n_samples = read_int(connection)
                    print(f'n_samples = {params.n_samples}')
                else:
                    params.w = read_float(connection)
                    n_params = read_int(connection)
                    if mode ==  modes['OPEN_LOOP']:
                        params.mode = 'OPEN_LOOP'
                    elif mode ==  modes['CLASSICAL']:
                        params.mode = 'CLASSICAL'
                    elif mode ==  modes['STATE_SPACE']:
                        params.mode = 'STATE_SPACE'
                    elif mode ==  modes['EXTENDED']:
                        params.mode = 'EXTENDED'
                    elif mode ==  modes['TEST']:
                        params.mode = 'TEST'                        
                    if n_params > 0:
                        parameters = []
                        for _ in range(n_params):
                            parameters.append(read_float(connection))
                        controller.set_params(parameters)
        connection.close()