class Rocket:
    weight = 0 #in Newtons
    drag = 0 #in Newtons
    liftoff_force = 0 #in Newtons
    x_angle = 0 #in radians
    y_angle = 0 #in radians

    def __init__(self, w, d, l_force):
        weight = w
        drag = d
        liftoff_force = l_force

    def adjustDirection(x, y):
        #run servos
        #run more servos
        x_angle += x #simulate the effect of servos moving
        y_angle += y #in reality would be replaced with imu readings

    def getX():
        return x_angle
    
    def getY():
        return y_angle
    
    
