import math

class Rocket:
    #physical measurements ----
    weight = 0 #in Newtons
    altitude = 0 #in feet
    cross_area = 0 #in m^2
    drag = 0 #in Newtons
    liftoff_force = 0 #in Newtons
    velocity_up = 0 # in m/s
    total_velocity = 0 #in m/s
    acceleration_up = 0 #in m/s^2
    x_wind = 0 #in m/s
    y_wind = 0 #in m/s
    mass = 0 #in kg
    
    time = 0 #in seconds
    nose_drag_cf = 0 #unitless
    air_density = 1.225 #kg/m^3 standard air density at sea level

    #rocket angling ----
    x_fin = 0 #in degrees
    y_fin = 0 #in degrees
    distance_cm_cp = -0.05 #distance from the cm(center of mass) to the cp(center of pressure)
    side_drag_cf = 0.9 #the drag coefficient of the side of the rocket(since wide blows from the side)
    side_area = 0 #total area of vertical cross-sectional area of rocket
    moment_slope_per_radian = -0.1 #change in moment coefficient/change in radians, units: 1/rad
    moment_reference_length = 0 #length as reference to keep moment coefficient unitless ;)
    x_angle = 0 #in radians
    y_angle = 0 #in radians
    x_angular_velocity = 0 #in rad/s
    y_angular_velocity = 0 #in rad/s

    #parachute ----
    parachute_drag_cf = 1.5 #unitless
    parachute_cross_area = 0.38484510006 # assuming 0.7m diameter
    parachute_on = False
    

    def __init__(self, w, l_force, cross, nose_drag, cm_cp, side_drag, vert_cross_area, mom_slop_rad, mom_ref_len): #weight, liftoff force, cross-sectional area
        #drag coefficient, cm->cp distance, side drag coefficient, vertical cross-sectional area, moment slope per radian, moment reference length
        self.weight = w
        self.mass = self.weight/9.81
        self.liftoff_force = l_force
        self.cross_area = cross
        self.nose_drag_cf = nose_drag
        self.distance_cm_cp = cm_cp
        self.side_drag_cf = side_drag
        self.side_area = vert_cross_area
        self.moment_slope_per_radian = mom_slop_rad
        self.moment_reference_length = mom_ref_len

    def adjustDirection(self, x, y):
        #run servos
        #run more servos

        self.x_fin = x #set new positions for proper adjustments
        self.y_fin = y

    def inputWind(self, x, y):
        self.x_wind = x
        self.y_wind = y

    def updateState(self, s, a, t, al, f, x_a, y_a): #input upward speed, time, altitude, lift force (by motors), x angle, y angle
        #---- calculate values of rocket
        totalVelocity = math.sqrt((self.velocity_up*math.tan(self.x_angle))**2 + (self.velocity_up*math.tan(self.y_angle))**2 + self.velocity_up**2) #full velocity, accounting for possible x & y differences
        self.drag = 0.5 * self.nose_drag_cf * self.air_density * self.cross_area * ((self.total_velocity+totalVelocity)/2)**2 #drag force = 0.5 * drag coefficient * density of air * cross-sectional area * total velocity^2

        self.liftoff_force = f

        self.velocity_up = s
        self.time = t
        self.altitude = al
        self.acceleration_up = a
        
        self.x_angle = x_a
        self.y_angle = y_a