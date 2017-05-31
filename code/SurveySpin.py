from StateQueue import StateQueue
from Stop import Stop
from Rotate import Rotate

class SurveySpin(StateQueue):
    def __init__(self, rover):
        super().__init__(rover)
        self.add(Stop(rover, 0.2))
        self.add(Rotate(rover, 360))