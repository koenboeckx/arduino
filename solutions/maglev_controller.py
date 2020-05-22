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
            self.k    = parameters[0]
            self.zero = parameters[1]
            self.pole = parameters[2]
            self.k2    = parameters[3]
            self.zero2 = parameters[4]
            self.type  = parameters[5]
            print(f'type = {int(self.type)}, k = {self.k:5.3f}, zero = {self.zero:5.3f}, pole = {self.pole:5.3f}, k2 = {self.k2:5.3f}, zero = {self.zero2:5.3f}')
        elif params.mode == 'STATE_SPACE':
            k1   = parameters[0]
            k2   = parameters[1]
            self.K = np.array([[k1, k2]])
            if len(parameters) == 4:
                l1   = parameters[2]
                l2   = parameters[3]
                self.observer.set_L(l1, l2)
        elif params.mode == 'EXTENDED':
            k1   = parameters[0]
            k2   = parameters[1]
            k3   = parameters[2]
            self.KE = np.array([[k1, k2, k3]])
            self.SE = 0.0
            if len(parameters) == 5:
                l1   = parameters[3]
                l2   = parameters[4]
                self.observer.set_L(l1, l2)

    def __call__(self, y):
        if params.mode == 'OPEN_LOOP':
            u_eff = params.w
            returns = [u_eff, y]
        elif params.mode == 'CLASSICAL':
            y = y - self.z0
            # zero static error
            e2 = params.w - y
            u2 = self.k2 * (e2 - self.zero2 * self.e2_prev) + self.u2_prev
            # Lead-Lag
            if self.type == 1.:
                e = u2 - y
            else:
                e = params.w - y
            u = self.k * (e - self.zero * self.e_prev) + self.pole * self.u_prev

            # anti-windup
            u = max(min(u, 20-self.I0), -self.I0)
            e = (u - self.pole * self.u_prev)/self.k + self.zero * self.e_prev
            u2 = e + y
            e2 = (u2 - self.u2_prev)/self.k2 + self.zero2 * self.e2_prev
            self.u2_prev = u2
            self.e2_prev = e2
            self.u_prev = u
            self.e_prev = e

            u_eff = u + self.I0
            returns = [u_eff, y]
        elif params.mode == 'STATE_SPACE':
            y = y - self.z0
            x_hat = self.observer(self.u_prev, y)
            u = params.w - self.K.dot(x_hat)
            self.u_prev = u

            u_eff = u + self.I0
            returns = [u, y, x_hat[0], x_hat[1]]
        
        elif params.mode == 'EXTENDED':
            y = y - self.z0
            x_hat = self.observer(self.u_prev, y)
            self.SE += y - params.w
            xE = np.append(x_hat, self.SE) 
            u = -self.KE.dot(xE)[0]
            self.u_prev = u

            u_eff = u + self.I0
            returns = [u, y, x_hat[0], x_hat[1]]
        else:
            u = 0.0
        #print(f"y = {y:5.3f}, e = {error:5.3f}, u = {u:5.3f}")
        u_eff = max(min(u_eff, 20), 0)
        return u_eff, returns

class Observer:
    def __init__(self):
        self.A = np.array([[1.0002, 0.02], [0.0224, 1.0002]])
        self.B = np.array([[-0.0001], [-0.0136]])
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
