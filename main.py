from mpc.Vessel import Vessel
from mpc.Controller import Controller

ctrl = Controller()
ctrl.run()
# ctrl.run_model_validation()


# TODO: sliding target
# TODO: optimize on other thread. Use the values till another optimization is done
# TODO: run inner loop of model with dt=0.1


