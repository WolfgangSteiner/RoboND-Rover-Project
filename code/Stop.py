class Stop(object):
    def __init__(self, rover, brake=1.0):
        self.rover = rover
        self.brake = brake


    def run(self):
        self.rover.brake = self.brake
        self.rover.throttle = 0.0
        self.rover.steer = 0.0


    def next(self):
        if abs(self.rover.vel) < 0.2:
            return None
        else:
            return self