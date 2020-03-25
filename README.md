# Part of the course ES323 - Control Systems project.

See the instruction video [here](https://youtu.be/RkP7duME04g)

## Requirements:
* matlab
* python 3 (preferably 3.6, although 3.7 might work as well but hasn't been tested)
* matplotlib (for visualization)

## General instructions:
1. download the files, either with `git clone` (preferred) or by downloading all the files in zip format (choose `Clone or download` in the top right corner)
1. Unzip the files (if needed). Open a command window and `cd` into the newly created directory.
1. Start the system simulator: `python3 system.py [bob|ship|maglev|bicopter]`.
Specifying the type of system to simulate is mandatory.
1. Start, in a new command window, the controller which implements the communication with the simulated system and matlab: `python3 controller.py [tcp_port]`. 
Students should only adapt the class `Controller`. This class takes the measurement `y` and produces the system input `u`.
1. add the folder `matlab_tools` to the matlab path and run the matlab script:
    * connect to `controller.py` with `arduino = tcpclient('localhost', tcp_port);`
    * set mode and reference value with `set_mode_params(arduino, mode, w, [])` where `mode` can be either `OPEN_LOOP`, `CLASSICAL` or `STATE_SPACE`. These same modes should also be defined in `Controller`.
    * get a response of the system with `[y, u] = get_response(arduino, w, n_samples)` where `y` are the measurements and `u` is the system input (generated by the controller).
    * reset the system to its initial state by using `reset_system(arduino)`
    * apply a disturbance (interpretation in system-specific) with `set_disturbance(arduino, dsiturbance)`.

## Specific instructions
All parameters of the systems can be found in the `.params` dictionary of each `System` object in `system.py`.

Measurement noise can be activated/deactivated by setting the `ADD_NOISE` flag to `True/False` in `system.py`
### Bicopter
Two rotors attached to the ends of pole that can rotate around its axis. It makes an angle `theta` with the horizontal.
* state vars = [`theta`, `omega`]
* init_state = [0.0, 0.0] - in radians and radians/sec
* limits on state: -10.0° < `theta` < 10.0°
* input = the difference in voltage `V` that is applied to the motors that drive both rotors
* limits on input: -10.0 < `V` < 10.0 
* measurement: `theta`
* disturbance = a torque `T` [Nm] applied to the rotation axis.

### Ship
A ship lying in a bassin of water. The mass at the end of a pole if mounted on top of the ship and can rotate (angle `phi`) around a vertical axis to steer the behavior of the ship around its axis (angle `theta`).
* state vars = [`theta`, `omega`]
* limits on state: -1.0 < `theta` < 1.0
* init_state = [0.0, 0.0] - in radians and radians/sec
* input = angle `phi` [radians]
* limits on input: -1.57 < `phi` < 1.57
* measurement: `theta`
* disturbance = an acceleration applied to the ship. Toggle this to simulate a push to the side of the ship.

### Ball On Beam
A small ball can roll over a beam. This beam can rotate around its axis and the rotation angle `alpha` can directly be set.
* state vars = [`x`, `v` ] - the position and speed of the ball
* limits on state: -1.0 < `x` < 1.0
* init_state = [1.0, 1.0]  - in [m] and [m/s]
* input = angle `alpha`
* limits on input: -10.0° < `alpha` < 10.0°
* measurement: position on beam `x` with a camera mounted above the setup - thus the projection of `x` in the horizontal plane.
* disturbance = an acceleration applied to the ball. Toggle this to simulate a push to the ball.

### Magnetic Levitation
A small iron ball is put inside a magnetic field. The strength of this magnetic field can be controlled by means of a current `I` through an inductance.
* state vars = [`z`, `v`] where `z` is the vertical distance ([cm]), measured from the top of the box in which the ball is confined. `v` is the vertical velocity.
* limits on state: 0.0 < `z` < 10.0
* init_state = [5.0, 0.0]
* input = current `I` [A]
* limits on input: 0.0 < `I` < 20.0
* disturbance: the `set_disturbance` function here limits the minimum value of `z`. This is needed to do the identification and parameter estimation of the system.
