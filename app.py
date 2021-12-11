from kinemaics import Leg

class app(Leg, Dynamixel):
    def __init__(self):
        Leg_L = Leg()
        Leg_R = Leg()
        DLX   = Dynamixel()
