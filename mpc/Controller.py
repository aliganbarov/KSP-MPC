

class Controller:

    def __init__(self, vessel):
        print("Controller was initialized")
        self.vessel = vessel

    def run(self):
        self.vessel.next_stage()

