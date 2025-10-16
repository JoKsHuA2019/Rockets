import Rocket_Model
import Wind
import matplotlib.pyplot as plt
import time
from simple_pid import PID

#------------------- rocket intializer ---------------------

rocket_weight = 0.550 * 9.81 #in N, kg * gravity = weight
rocket_drag_coefficient = 0.75 #drag coefficient
rocket_total_liftoff_force = 80 #motor force
rocket_cross_area = 0.007854 #in m^2, assuming 25mm radius

#-------------- PID setup -------------

pid_x = PID(1, 0.1, 0.05, setpoint=0)
pid_y = PID(1, 0.1, 0.05, setpoint=0)

pid_x.sample_time = 0.01
pid_y.sample_time = 0.01

myRocket = Rocket_Model.Rocket(rocket_weight, rocket_total_liftoff_force, rocket_cross_area, rocket_drag_coefficient)

def run_PID():
    output_x = pid_x(myRocket.x_angle)
    output_y = pid_y(myRocket.y_angle)

    myRocket.adjustDirection(output_x, output_y)

#---------------matplotlib setup----------------

plt.plot([1,2,3,4])


#----------------Force Function (of motors)-----------------
def motor_force(t):
    new_force = -t**2/10 + 5
    if (new_force < 0):
        new_force = 0
    return new_force


#----------------- Simulation Setup -------------------

#wind conditions here
wind_speed = 0

#run the simulation by the time elapsed (time.monotonic())
start_time = time.time()
wind_time = time.time()
update_time = time.time()


#----------------- calculate whether to deploy parachute

#----------------- Simulation --------------------

while True:
    run_PID()
    if (time.time() > wind_time + 0.25): #makes sure the wind only changes in 0.25 second spacing
        wind_speed = Wind.next_wind(wind_speed)
        wind_time = time.time()

    if (time.time() > update_time + 0.05):
        time_elapsed_since_update = time.time() - update_time
        total_elapsed_time = time.time() - start_time

        update_force = motor_force(total_elapsed_time)
        curr_accel = update_force/myRocket.weight

        update_speed = myRocket.velocity_up + time_elapsed_since_update*((myRocket.acceleration_up+curr_accel)/2) #v=at, a = v/t
        update_altitude = myRocket.altitude + (update_speed + myRocket.velocity_up)*time_elapsed_since_update/2 #x = vt
        
        if (myRocket.parachute_on):
            update_speed -= myRocket.air_density*myRocket.parachute_drag_coefficient*myRocket.parachute_cross_area*((myRocket.velocity_up+update_speed)/2) * time_elapsed_since_update / (myRocket.weight/9.81) #v = at, a  = F/m, F = (airdensity)(dragcoefficient)(parachutecrosssec)(parachutevelocity)
        
        myRocket.updateState(update_speed, update_time, update_altitude, update_force)
        update_time = time.time()


    