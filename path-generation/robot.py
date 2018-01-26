#Not really sure if this deserves it's own class
class Robot:
    def __init__(self,width,velocity,acceleration):
        self.width = width
        self.max_velocity = velocity
        self.max_acceleration = acceleration

    def __str__(self):
        title =  " Robot Attributes"
        width       =  " Width       : " + str(self.width) + " feet"
        velocity    =  " Velocity    : " + str(self.max_velocity) + " feet per second"
        acceleration =  " Acceleration: " + str(self.max_acceleration) + " feet per second squared"
        return title + "\n" + width + "\n" + velocity + "\n" + acceleration
