import Rocket_Model
import Wind
import matplotlib.pyplot as plt
import time
from simple_pid import PID
import math

#------------------- rocket intializer ---------------------
rocket_radius = 0.025
rocket_height = 0.8

rocket_weight = 0.550 * 9.81 #in N, kg * gravity = weight
rocket_drag_coefficient = 0.75 #drag coefficient
rocket_total_liftoff_force = 80 #motor force
rocket_cross_area = math.pi * rocket_radius**2 #in m^2

distance_cm_cp = 0.05 #distance from the cm(center of mass) to the cp(center of pressure)
side_drag_cf = 0.9 #the drag coefficient of the side of the rocket(since wide blows from the side)
side_area = rocket_height * rocket_radius*2 #total area of vertical cross-sectional area of rocket (assuming 0.8m by 0.05m) in m^2
moment_slope_per_radian = 1.5 #change in moment coefficient/change in radians, units: 1/rad
moment_reference_length = 0.05 #length as reference to keep moment coefficient unitless ;)
moment_of_inertia = 0.5 * (rocket_weight/9.81) * rocket_radius**2 #calculated assuming rocket is a perfect cylinder

#-------------- PID setup -------------

pid_x = PID(0.2, 0.03, 0.01, setpoint=0)
pid_y = PID(0.2, 0.03, 0.01, setpoint=0)

pid_x.output_limits = (-0.35, 0.35)
pid_y.output_limits = (-0.35, 0.35)

pid_x.sample_time = 0.01
pid_y.sample_time = 0.01

myRocket = Rocket_Model.Rocket(rocket_weight, rocket_total_liftoff_force, rocket_cross_area, rocket_drag_coefficient, distance_cm_cp, side_drag_cf, side_area, moment_slope_per_radian, moment_reference_length)

def run_PID():
    output_x = pid_x(myRocket.x_angle)
    output_y = pid_y(myRocket.y_angle)

    myRocket.adjustDirection(output_x, output_y)

#---------------matplotlib setup----------------

plt.ion()

fig, axes = plt.subplots(2, 3, figsize=(13,9.5), constrained_layout = True)
(ax1, ax2, ax3), (ax4, ax5, ax6) = axes
plt.tight_layout()

time_data = []
velocity_data = []
accel_data = []
x_fin_data = []
y_fin_data = []
altitude_data = []
wind_data = []

line1, = ax1.plot([], [])
line2, = ax2.plot([], [])
line3, = ax3.plot([], [])
line4, = ax4.plot([], [])
line5, = ax5.plot([], [])
line6, = ax6.plot([], [])

ax1.set_title('Rocket Velocity vs Time')
ax1.set_xlabel('time [s]')
ax1.set_ylabel('velocity [m/s]')

ax2.set_title('Rocket Acceleration vs Time')
ax2.set_xlabel('time [s]')
ax2.set_ylabel('acceleration [m/s^2]')

ax3.set_title('Rocket x fin pos vs Time')
ax3.set_xlabel('time [s]')
ax3.set_ylabel('x fin position [degree]')

ax4.set_title('Rocket y fin pos vs Time')
ax4.set_xlabel('time [s]')
ax4.set_ylabel('y fin position [degree]')

ax5.set_title('Rocket altitude vs Time')
ax5.set_xlabel('time [s]')
ax5.set_ylabel('altitude [m]')

ax6.set_title('Wind Speed vs Time')
ax6.set_xlabel('time [s]')
ax6.set_ylabel('wind speed [m/s]')

#----------------Force Function (of motors)-----------------
def motor_force(t):
    new_force = -(t**2) + 35
    if (new_force < 0):
        new_force = 0
    #print(new_force)
    return new_force

#----------------- Simulation Setup -------------------

#wind conditions here
wind_speed_x = 0
wind_speed_y = 0

#stats
highest_altitude = 0
highest_velocity = 0
total_time = 0

#drift stats
x_drift_distance = 0
y_drift_distance = 0

#run the simulation by the time elapsed (time.monotonic())
start_time = time.time()
wind_time = time.time()
update_time = time.time()

#-----------------calculate whether to deploy parachute---------------

#----------------- Simulation --------------------

#still has a problem of having very oscillating accel/velo when liftoff force is high
#there is also a trend for some reason that the fins only move in the negative direction
#maybe add a second PID for angling during descent, not very needed though

while True:
    if (myRocket.velocity_up > 0):
        run_PID()
    else:
        myRocket.parachute_on = True
        myRocket.x_fin = 0
        myRocket.y_fin = 0

    if (time.time() > wind_time + 0.25): #makes sure the wind only changes in 0.25 second spacing
        wind_speed_x = Wind.next_wind(wind_speed_x)
        wind_speed_y = Wind.next_wind(wind_speed_y)

        myRocket.inputWind(wind_speed_x, wind_speed_y)

        wind_time = time.time()

    if (time.time() > update_time + 0.01):
        time_elapsed_since_update = time.time() - update_time
        total_elapsed_time = time.time() - start_time

        #subtract the drag with same sign as velocity (in order to make sure that it is oppositin force)
        update_force_up = motor_force(total_elapsed_time) - myRocket.weight - math.copysign(myRocket.drag, myRocket.velocity_up)

        curr_accel = update_force_up/myRocket.mass

        update_velocity_up = myRocket.velocity_up + time_elapsed_since_update*((myRocket.acceleration_up+curr_accel)/2) #v=at, a = v/t
        x_drift_distance = myRocket.x_drift + time_elapsed_since_update*((abs(update_velocity_up+myRocket.velocity_up)*math.tan(myRocket.x_angle))/2)
        y_drift_distance = myRocket.y_drift + time_elapsed_since_update*((abs(update_velocity_up+myRocket.velocity_up)*math.tan(myRocket.y_angle))/2)
        update_altitude = myRocket.altitude + (update_velocity_up + myRocket.velocity_up)*time_elapsed_since_update/2 #x = vt
        
        if (myRocket.parachute_on):
            update_velocity_up -= myRocket.air_density*myRocket.parachute_drag_cf*myRocket.parachute_cross_area*((myRocket.velocity_up+update_velocity_up)/2) * time_elapsed_since_update / myRocket.mass #v = at, a  = F/m, F = (airdensity)(dragcoefficient)(parachutecrosssec)(parachutevelocity)

        #-----rocket angling depends on wind speed, assumed to be blowing horizontally
        #this translates to drag force, and from there, we can calculate acceleration,
        #moment, and finally, resultant angle. This angle will be continuously updated
        #as the loop goes on, every 0.01 seconds, modifying the original-----

        #to calculate, we use dynamic tilt equation I(d^2theta/dt^2) = torque(wind) - torque(restoring)

        #first, calculate the torque of rocket restoration
    #    x_restoring_torque = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.x_wind**2) * side_area * moment_reference_length * moment_slope_per_radian * math.tanh(myRocket.x_wind/((myRocket.velocity_up+update_velocity_up)/2))
    #    y_restoring_torque = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.y_wind**2) * side_area * moment_reference_length * moment_slope_per_radian * math.tanh(myRocket.y_wind/((myRocket.velocity_up+update_velocity_up)/2))

        #more linear restoring torque cause the other one was very inconsistent
        x_restoring_torque = -moment_slope_per_radian * myRocket.x_angle
        y_restoring_torque = -moment_slope_per_radian * myRocket.y_angle


        #then, we need to calculate the force of the wind on the rocket
        x_wind_force = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.x_wind**2) * side_drag_cf * side_area
        y_wind_force = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.y_wind**2) * side_drag_cf * side_area
        #use the force of wind to calculate wind torque
        x_wind_torque = x_wind_force * distance_cm_cp
        y_wind_torque = y_wind_force * distance_cm_cp
        #calculate angular acceleration
        x_wind_angular_accel = (x_wind_torque + x_restoring_torque) / moment_of_inertia
        y_wind_angular_accel = (y_wind_torque + y_restoring_torque) / moment_of_inertia
        #dampen angular accel to prevent oscillations
        angular_damping = 2.5
        x_wind_angular_accel -= angular_damping * myRocket.x_angular_velocity
        y_wind_angular_accel -= angular_damping * myRocket.y_angular_velocity
        #update angular velocity
        x_angular_velo = myRocket.x_angular_velocity + x_wind_angular_accel * time_elapsed_since_update
        y_angular_velo = myRocket.y_angular_velocity + y_wind_angular_accel * time_elapsed_since_update

        #limit the angluar velocities to get no more blowup
        x_angular_velo = max(min(x_angular_velo, 15), -15)
        y_angular_velo = max(min(y_angular_velo, 15), -15)

        #update angle
        update_x_angle = myRocket.x_angle + ((x_angular_velo + myRocket.x_angular_velocity)/2) * time_elapsed_since_update
        update_y_angle = myRocket.y_angle + ((y_angular_velo + myRocket.y_angular_velocity)/2) * time_elapsed_since_update
        #account for fin correction
        update_x_angle -= myRocket.x_fin/15
        update_y_angle -= myRocket.y_fin/15

        #print("x_angle:", update_x_angle * 180/math.pi)
        #print("y_angle:", update_y_angle * 180/math.pi)

        myRocket.updateState(update_velocity_up, curr_accel, update_time, update_altitude, update_force_up, update_x_angle, update_y_angle, x_drift_distance, y_drift_distance)
        update_time = time.time()


        #display graphs
        if (time.time()%0.1 <0.01):
            time_data.append(total_elapsed_time)
            velocity_data.append(myRocket.velocity_up)
            accel_data.append(myRocket.acceleration_up)
            x_fin_data.append(myRocket.x_fin)
            y_fin_data.append(myRocket.y_fin)
            altitude_data.append(myRocket.altitude)
            wind_data.append(math.sqrt(myRocket.x_wind**2 + myRocket.y_wind**2)*(myRocket.x_wind*myRocket.y_wind)/(abs(myRocket.x_wind*myRocket.y_wind)+0.0001)) #the two statements at the end make sure that the wind direction (+/-) is correct

            line1.set_data(time_data, velocity_data)
            line2.set_data(time_data, accel_data)
            line3.set_data(time_data, x_fin_data)
            line4.set_data(time_data, y_fin_data)
            line5.set_data(time_data, altitude_data)
            line6.set_data(time_data, wind_data)

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            ax3.relim()
            ax3.autoscale_view()
            ax4.relim()
            ax4.autoscale_view()
            ax5.relim()
            ax5.autoscale_view()
            ax6.relim()
            ax6.autoscale_view()

            plt.draw()
            plt.pause(0.01)

        highest_altitude = max(highest_altitude, myRocket.altitude)
        highest_velocity = max(highest_velocity, myRocket.velocity_up)
        total_time = time.time() - start_time

        if (myRocket.altitude < 0):
            #print stats
            print("highest altitude reached: ", highest_altitude, "meters")
            print("highest velocity reached: ", highest_velocity, "meters per second")
            print("total flight time: ", total_time, "seconds")
            print("total x direction drift: ", x_drift_distance, "meters")
            print("total y direction drift: ", y_drift_distance, "meters")
            break
