from simple_pid import PID
import Rocket_Functions

pid_x = PID(1, 0.1, 0.05, setpoint=0)
pid_y = PID(1, 0.1, 0.05, setpoint=0)

pid_x.sample_time = 0.01
pid_y.sample_time = 0.01

#rocket intializer
rocket_weight = 0.550 * 9.81 #in N, kg * gravity = weight
rocket_drag_coefficient = 0.75 #drag coefficient
rocket_total_liftoff_force = 80 #motor force
rocket_cross_area = 0.007854 #in m^2, assuming 25mm radius

myRocket = Rocket_Functions.Rocket(rocket_weight, rocket_total_liftoff_force, rocket_cross_area, rocket_drag_coefficient)

def run_PID():
    output_x = pid_x(myRocket.getX())
    output_y = pid_y(myRocket.getY())

    myRocket.adjustDirection(output_x, output_y)