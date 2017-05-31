from perception import is_obstacle_ahead
from TurnToBearing import TurnToBearing

class StopAndTurn(object):
    def __init__(self, rover):
        self.rover = rover
        self.is_clear = False

    def run(self):
        if self.rover.vel > 0.2:
            self.rover.stop()
        else:
            self.rover.brake = 0
            self.rover.steer = -15


    def next(self):
        if not is_obstacle_ahead(self.rover):
            if not self.is_clear:
                self.is_clear = True
                return TurnToBearing(self.rover, -15)
            else:
                self.rover.steer = 0
                return None
        else:
            return self