

class Settings:
    # Kerbin mass
    M = 5.2915158e22
    # Kerbin radius
    R = 600000
    # Kerbin gravity constant
    G = 6.67408e-11

    def __init__(self):
        self.LAUNCH = 0
        self.ASCEND = 1
        self.DESCEND = 2
        self.LANDED = 3

