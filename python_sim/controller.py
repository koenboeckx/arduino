from multiprocessing.connection import Client
import socket
import threading, time

Ts = 0.01

def get_measurement(conn):
    conn.send('measure')
    y = conn.recv()
    return y

def set_u(conn, u):
    conn.send('command')
    conn.send(u)

class Controller:
    "Simple proportional controller"
    def __init__(self):
        self.K = 0.3;
    
    def set_gain(self, K):
        self.K = K

    def __call__(self, y):
        return self.K*y

def system_comms(controller):
    address = ('localhost', 6000)
    with Client(address, authkey=b'secret') as conn:
        while True:
            y = get_measurement(conn)
            #print(y)
            u = controller(y)
            set_u(conn, u)
            time.sleep(Ts)

def matlab_comms(controller):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('localhost', 6001)
    sock.bind(server_address)
    sock.listen(1)

    while True:
        # Wait for a connection
        connection, client_address = sock.accept()
        print(connection, client_address)
        try:
            while True:
                data = connection.recv(100)
                if data:
                    print(int(data[0]))
                    K = int(data[0])
                    controller.set_gain(K)
        finally:
            connection.close()

if __name__ == '__main__':
    controller = Controller()

    sys_comms_thread = threading.Thread(target=system_comms, args=(controller, ))
    sys_comms_thread.start()

    matlab_comms_thread = threading.Thread(target=matlab_comms, args=(controller, ))
    matlab_comms_thread.start()
