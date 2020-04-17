from mpc.Vessel import Vessel
from mpc.Controller import Controller

vessel = Vessel()
ctrl = Controller(vessel)
ctrl.run()


