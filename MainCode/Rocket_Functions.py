import math

class Rocket:
    weight = 0 #in Newtons
    altitude = 0 #in feet
    cross_area = 0 #in m^2
    drag = 0 #in Newtons
    liftoff_force = 0 #in Newtons
    velocity_up = 0 # in m/s
    acceleration_up = 0 #in m/s^2
    x_angle = 0 #in radians
    y_angle = 0 #in radians
    time = 0 #in seconds
    drag_cofficient = 0 #unitless
    air_density = 1.225 #kg/m^3 standard air density at sea level

    def __init__(self, w, l_force, cross, drag_cf):
        self.weight = w
        self.liftoff_force = l_force
        self.cross_area = cross
        self.drag_coefficient = drag_cf

    def adjustDirection(self, x, y):
        #run servos
        #run more servos
        self.x_angle += x #simulate the effect of servos moving
        self.y_angle += y #in reality would be replaced with imu readings

    def updateState(self, x, y, s, t, al, a): #input x adjustment, y adjustment, upward speed, time, altitude, acceleration (by motors)
        self.adjustDirection(x, y)
        self.velocity_up = s
        self.time = t
        self.altitude = al
        self.acceleration_up = a
        totalVelocity = math.sqrt((self.velocity_up*math.tan(x))**2 + (self.velocity_up*math.tan(y))**2 + self.velocity_up**2) #full velocity, accounting for possible x & y differences
        self.drag = 0.5 * self.drag_coefficient * self.air_density * self.cross_area * totalVelocity**2 #drag force = 0.5 * drag coefficient * density of air * cross-sectional area * total velocity^2
        self.liftoff_force -= self.drag + self.weight

    def getX(self):
        return self.x_angle
    
    def getY(self):
        return self.y_angle
    
    def getUpwardVelocity(self):
        return self.velocity_up
    