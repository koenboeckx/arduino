ADD_NOISE = True

class System:
    def __init__(self, T):
        self.T = T # sampling period / step size
        self.t = 0 # time
        self.state = self.init_state()
        self.u = 0.0 # keep track of last given command
        self.params = {} # container for system parameters
        self.disturbance = 0.0 # use depends on application
    
    def get_state(self):
        return self.state[0], self.state[1]

    def set_u(self, u):
            self.u = self._constrain_input(u)
    
    def step(self):
        #self.state += self.T * self.deriv(self.t, self.state, self.u)  # Euler scheme
        self.state += self._rk4(self.t, self.state, self.u)             # Runge-Kutta 4
        self.t += self.T
        self._constrain_state()

    def _rk4(self, t, state, u):
        """Implements Runge-Kutta-4 integration scheme"""
        h = self.T
        k1 = h * self.deriv(t, state, u)
        k2 = h * self.deriv(t + h/2, state + k1/2, u)
        k3 = h * self.deriv(t + h/2, state + k2/2, u)
        k4 = h * self.deriv(t + h, state + k3, u)
        return 1/6*(k1 + 2*k2 + 2*k3 + k4)

    def init_state(self):
        raise NotImplementedError
       
    def get_measurement(self):
        raise NotImplementedError

    def _constrain_input(self, u):
        raise NotImplementedError 
    
    def _constrain_state(self):
        raise NotImplementedError
    
    def deriv(self, t, x, u):
        """Computes first derivative"""
        raise NotImplementedError
