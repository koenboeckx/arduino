import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 
from utilities import *
#### ------ only change Controller class ----------- ####

class Controller:
    "Your Controller"
    def __init__(self):
        self.k = 8.0

        self.u_prev = 0.0
        self.prev_error = 0.0

        self.u3_prev = 0.0
        self.prev_error3 = 0.0

        self.observer = Observer()
        self.K = np.array([[-1.3064, -1.3463]])

        self.SE = 0.0
        self.KE = np.array([[-2.7216,   -3.4286,   -0.0350]])

        self.max_alpha = 0.2
    
    def set_params(self, parameters):
        "params from matlab set_mode_params"
        if params.mode == 'CLASSICAL':
            self.k1   = parameters[0]
            self.k2   = parameters[1]
            self.zero = parameters[2]
            self.pole = parameters[3]
            self.k3   = parameters[4]
            self.zero2 = parameters[5]
            print(f'k1 = {self.k1}, k2 = {self.k2}, zero = {self.zero}, pole = {self.pole}')
            self.u_prev = 0.0
            self.prev_error = 0.0
            self.u3_prev = 0.0
            self.prev_error3 = 0.0

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
            if len(parameters) == 5:
                l1   = parameters[3]
                l2   = parameters[4]
                self.observer.set_L(l1, l2)

    def __call__(self, y):
        returns = []
        if params.mode == 'OPEN_LOOP':
            u = params.w
            returns = [u, y]
        elif params.mode == 'CLASSICAL':
            # 1. Integrating loop
            e3 = params.w - y
            u3 = self.k3 * (e3 - self.zero2 * self.prev_error3) + self.u3_prev
            
            # 2. Lead-Lag
            e2 = u3 - y
            u2 = self.k2 * (e2 - self.zero * self.prev_error) + self.pole * self.u_prev
            
            # 3. proportional
            e1 = u2 - y
            u  = self.k1 * e1
            #print(f"e1={e1:3.2f}, u2={u2:3.2f}, e2={e2:3.2f}, u3={u3:3.2f}, e3={u3:3.2f}")
            # anti-windup
            u = min(max(u, -self.max_alpha), self.max_alpha)
            e1 = u/self.k1
            u2 = e1 + y
            e2 = (u2 - self.pole * self.u_prev)/self.k2 + self.zero * self.prev_error
            u3 = e2 + y
            e3 = (u3 - self.u3_prev)/self.k3 + self.zero2 * self.prev_error3
            
            self.u3_prev = u3
            self.prev_error3 = e3
            self.u_prev = u2
            self.prev_error = e2

            returns = [u, y]
        elif params.mode == 'STATE_SPACE':
            u = params.w
            x_hat = self.observer(self.u_prev, y)
            u = params.w - self.K.dot(x_hat)
            returns = [u, y, x_hat[0], x_hat[1]]
            self.u_prev = u
        elif params.mode == 'EXTENDED':
            x_hat = self.observer(self.u_prev, y)
            self.SE += y - params.w
            xE = np.append(x_hat, self.SE) 
            u = -self.KE.dot(xE)[0]
            returns = [u, y, x_hat[0], x_hat[1]]
            self.u_prev = u
        else:
            u = 0.0
        return u, returns

class Observer:
    def __init__(self):
        self.A = np.array([[1., 0.05], [0., 1.]])
        self.B = np.array([[-0.0068], [-0.2732]])
        self.C = np.array([[1., 0.]])
        self.L = np.array([[0.39], [0.76]])
        self.x = np.array([[1.], [0.]])
    
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
