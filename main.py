from mpc.Vessel import Vessel
from mpc.Controller import Controller

ctrl = Controller()
# ctrl.run()
# ctrl.run_model_validation()


for lower_alt in range(1000, 20001, 4000):
    for upper_alt in range(1000, 20001, 4000):
        for vel in range(50, 201, 50):
            if vel > 50 and lower_alt == upper_alt:
                continue
            ctrl = Controller()
            ctrl.run_data_gathering(lower_alt, upper_alt, vel)


