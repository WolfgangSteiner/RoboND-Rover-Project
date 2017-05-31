class StateQueue(object):
    def __init__(self, rover):
        self.rover = rover
        self.queue = []


    def add(self, action):
        self.queue.append(action)


    def run(self):
        print(self.current_task())
        self.current_task().run()


    def current_task(self):
        return self.queue[0] if len(self.queue) else None


    def next(self):
        next_task = self.current_task().next()

        if next_task is None:
            self.queue.pop(0)
        elif next_task is not self.current_task():
            self.queue.insert(0, next_task)

        if self.current_task():
            return self
        else:
            return None

