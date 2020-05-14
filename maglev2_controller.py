import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 
from utilities import *

#### ------ only change Controller class ----------- ####

class Controller:
    "Your Controller"
    def __init__(self):
        self.I0 = 14.6
        self.z0 = 5.0
        self.k = 8.0

        self.u_prev  = 0.0
        self.e_prev  = 0.0
        self.u2_prev = 0.0
        self.e2_prev = 0.0

        self.observer = Observer()
        self.K = np.array([[-1.3064, -1.3463]])

        self.SE = 0.0
        self.KE = np.array([[-2.7216,   -3.4286,   -0.0350]])
    
    def set_params(self, parameters):
        "params from matlab set_mode_params"
        if params.mode == 'OPEN_LOOP':
            if len(parameters) == 2:
                self.I0 = parameters[0]
                self.z0 = parameters[1]
                print(f"z0 = {self.z0}, I0 = {self.I0}")
        elif params.mode == 'CLASSICAL':
            pass
        elif params.mode == 'STATE_SPACE':
            k1   = parameters[0]
            k2   = parameters[1]
            self.K = np.array([[k1, k2]])
            if len(parameters) == 4:
                l1   = parameters[2]
                l2   = parameters[3]
                self.observer.set_L(l1, l2)
        elif params.mode == 'EXTENDED':
            pass

    def __call__(self, y):
        if params.mode == 'OPEN_LOOP':
            u_eff = params.w
            returns = [u_eff, y]
        elif params.mode == 'CLASSICAL':
            pass
        elif params.mode == 'STATE_SPACE':
            y = y - self.z0
            x_hat = self.observer(self.u_prev, y)
            u, u_eff = 0, params.w # ... To Do 
            self.u_prev = u
            returns = [u, y, x_hat[0], x_hat[1]]
        elif params.mode == 'EXTENDED':
            pass
        else:
            u = 0.0
        u_eff = max(min(u_eff, 20), 0)
        return u_eff, returns

class Observer:
    def __init__(self):
        self.A = np.array([[1.0002, 0.05], [0.0818, 1.0002]])
        self.B = np.array([[-0.0010], [-0.0417]])
        self.C = np.array([[1., 0.]])
        self.L = np.array([[0.2904], [1.0755]])
        self.x = np.array([[5.], [0.]])
    
    def set_L(self, l1, l2):
        self.L[0] = l1
        self.L[1] = l2
    
    def __call__(self, u, y):
        self.x = self.A.dot(self.x) + self.B.dot(u) + self.L*(y - self.C.dot(self.x))
        return self.x

#### ------ don't change anything below ----------- ####

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
