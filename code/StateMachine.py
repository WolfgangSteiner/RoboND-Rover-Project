class StateMachine(object):
    def __init__(self, rover, *args):
        self.rover = rover
        self.state_stack = list(args)


    def current_state(self):
        return self.state_stack[0]


    def run(self):
        if len(self.state_stack) == 0:
            print("DONE!!!")
            return

        print(self.current_state())

        self.current_state().run()

        next_state = self.current_state().next()

        if next_state is self.current_state():
            pass
        elif next_state is None:
            self.state_stack.pop(0)
        else:
            self.state_stack.insert(0, next_state)


    def push_front(self, action):
        self.state_stack.insert(0, action)

