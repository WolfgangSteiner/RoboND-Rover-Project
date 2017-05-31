class PickupRock(object):
    def __init__(self, rover):
        self.rover = rover
        self.did_issue_command = False


    def run(self):
        if not self.did_issue_command:
            self.rover.pick_up = True
            self.did_issue_command = True


    def next(self):
        if self.rover.near_sample:
            return self
        else:
            return None
