from multiprocessing.connection import Client
import time

Ts = 0.1

def get_measurement(conn):
    conn.send('measure')
    y = conn.recv()
    return y

def set_u(conn, u):
    conn.send('command')
    conn.send(u)

def controller(y):
    return 0.1*y

address = ('localhost', 6000)
with Client(address, authkey=b'secret') as conn:
    while True:
        y = get_measurement(conn)
        print(y)
        u = controller(y)
        set_u(conn, u)
        time.sleep(Ts)

