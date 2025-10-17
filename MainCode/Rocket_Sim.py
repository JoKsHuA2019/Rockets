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

distance_cm_cp = -0.05 #distance from the cm(center of mass) to the cp(center of pressure)
side_drag_cf = 0.9 #the drag coefficient of the side of the rocket(since wide blows from the side)
side_area = rocket_height * rocket_radius*2 #total area of vertical cross-sectional area of rocket (assuming 0.8m by 0.05m) in m^2
moment_slope_per_radian = -0.1 #change in moment coefficient/change in radians, units: 1/rad
moment_reference_length = 0.05 #length as reference to keep moment coefficient unitless ;)
moment_of_inertia = 0.5 * (rocket_weight/9.81) * rocket_radius**2 #calculated assuming rocket is a perfect cylinder

#-------------- PID setup -------------

pid_x = PID(1, 0.1, 0.05, setpoint=0)
pid_y = PID(1, 0.1, 0.05, setpoint=0)

pid_x.sample_time = 0.01
pid_y.sample_time = 0.01

myRocket = Rocket_Model.Rocket(rocket_weight, rocket_total_liftoff_force, rocket_cross_area, rocket_drag_coefficient, distance_cm_cp, side_drag_cf, side_area, moment_slope_per_radian, moment_reference_length)

def run_PID():
    output_x = pid_x(myRocket.x_angle)
    output_y = pid_y(myRocket.y_angle)

    myRocket.adjustDirection(output_x, output_y)

#---------------matplotlib setup----------------

plt.ion()

fig, axes = plt.subplots(2, 2, layout='constrained')
(ax1, ax2), (ax3, ax4) = axes
plt.tight_layout()

time_data = []
velocity_data = []
accel_data = []
x_fin_data = []
y_fin_data = []

line1, = ax1.plot([], [])
line2, = ax2.plot([], [])
line3, = ax3.plot([], [])
line4, = ax4.plot([], [])

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

#----------------Force Function (of motors)-----------------
def motor_force(t):
    new_force = -(t**2) + 60
    if (new_force < 0):
        new_force = 0
    print(new_force)
    return new_force

#----------------- Simulation Setup -------------------

#wind conditions here
wind_speed_x = 0
wind_speed_y = 0

#run the simulation by the time elapsed (time.monotonic())
start_time = time.time()
wind_time = time.time()
update_time = time.time()

#-----------------calculate whether to deploy parachute---------------

#----------------- Simulation --------------------

while True:
    run_PID()
    if (time.time() > wind_time + 0.25): #makes sure the wind only changes in 0.25 second spacing
        wind_speed_x = Wind.next_wind(wind_speed_x)
        wind_speed_y = Wind.next_wind(wind_speed_y)

        myRocket.inputWind(wind_speed_x, wind_speed_y)

        wind_time = time.time()

    if (time.time() > update_time + 0.01):
        time_elapsed_since_update = time.time() - update_time
        total_elapsed_time = time.time() - start_time

        update_force_up = motor_force(total_elapsed_time) - (myRocket.weight + myRocket.drag)
        if (myRocket.velocity_up <= 0): 
            update_force_up = motor_force(total_elapsed_time) - myRocket.weight + myRocket.drag

        curr_accel = update_force_up/myRocket.weight

        update_speed = myRocket.velocity_up + time_elapsed_since_update*((myRocket.acceleration_up+curr_accel)/2) #v=at, a = v/t
        update_altitude = myRocket.altitude + (update_speed + myRocket.velocity_up)*time_elapsed_since_update/2 #x = vt
        
        if (myRocket.parachute_on):
            update_speed -= myRocket.air_density*myRocket.parachute_drag_cf*myRocket.parachute_cross_area*((myRocket.velocity_up+update_speed)/2) * time_elapsed_since_update / myRocket.mass #v = at, a  = F/m, F = (airdensity)(dragcoefficient)(parachutecrosssec)(parachutevelocity)

        #-----rocket angling depends on wind speed, assumed to be blowing horizontally
        #this translates to drag force, and from there, we can calculate acceleration,
        #moment, and finally, resultant angle. This angle will be continuously updated
        #as the loop goes on, every 0.01 seconds, modifying the original-----

        #to calculate, we use dynamic tilt equation I(d^2theta/dt^2) = torque(wind) - torque(restoring)
        #first, calculate the torque of rocket restoration
        x_restoring_torque = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.x_wind**2) * side_area * moment_reference_length * moment_slope_per_radian * math.tanh(myRocket.x_wind/((myRocket.velocity_up+update_speed)/2))
        y_restoring_torque = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.y_wind**2) * side_area * moment_reference_length * moment_slope_per_radian * math.tanh(myRocket.y_wind/((myRocket.velocity_up+update_speed)/2))
        #then, we need to calculate the force of the wind on the rocket
        x_wind_force = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.x_wind**2) * side_drag_cf * side_area
        y_wind_force = 0.5 * myRocket.air_density * (myRocket.velocity_up**2 + myRocket.y_wind**2) * side_drag_cf * side_area
        #use the force of wind to calculate wind torque
        x_wind_torque = x_wind_force * distance_cm_cp
        y_wind_torque = y_wind_force * distance_cm_cp
        #calculate angular acceleration
        x_wind_angular_accel = (x_wind_torque - x_restoring_torque) / moment_of_inertia
        y_wind_angular_accel = (y_wind_torque - y_restoring_torque) / moment_of_inertia
        #update angular velocity
        x_angular_velo = myRocket.x_angular_velocity + x_wind_angular_accel * time_elapsed_since_update
        y_angular_velo = myRocket.y_angular_velocity + y_wind_angular_accel * time_elapsed_since_update
        #update angle
        update_x_angle = myRocket.x_angle + ((x_angular_velo + myRocket.x_angular_velocity)/2) * time_elapsed_since_update
        update_y_angle = myRocket.y_angle + ((y_angular_velo + myRocket.y_angular_velocity)/2) * time_elapsed_since_update
        #account for fin correction
        update_x_angle += myRocket.x_fin/1000
        update_y_angle += myRocket.y_fin/1000

        myRocket.updateState(update_speed, curr_accel, update_time, update_altitude, update_force_up, update_x_angle, update_y_angle)
        update_time = time.time()


        #display graphs
        if (time.time()%0.1 <0.01):
            time_data.append(time.time())
            velocity_data.append(myRocket.velocity_up)
            accel_data.append(myRocket.acceleration_up)
            x_fin_data.append(myRocket.x_fin)
            y_fin_data.append(myRocket.y_fin)

            line1.set_data(time_data, velocity_data)
            line2.set_data(time_data, accel_data)
            line3.set_data(time_data, x_fin_data)
            line4.set_data(time_data, y_fin_data)

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            ax3.relim()
            ax3.autoscale_view()
            ax4.relim()
            ax4.autoscale_view()

            plt.draw()
            plt.pause(0.01)
