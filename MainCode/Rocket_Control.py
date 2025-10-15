from simple_pid import PID
import Rocket_Functions

pid = PID(1, 0.1, 0.05, setpoint=0)

#rocket intializer
rocket_weight = 0.550 * 9.81 #kg * gravity = weight
rocket_drag = 0.75 #drag coefficient
rocket_total_liftoff_force = 80 #motor force
myRocket = Rocket_Functions()