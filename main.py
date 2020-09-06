from mpc.Vessel import Vessel
from mpc.Controller import Controller

ctrl = Controller('20K')
ctrl.run_landing()
# ctrl.run_model_validation()
# ctrl.run_pid_tuning('Roll')


'''
for lower_alt in range(1000, 20001, 4000):
    for upper_alt in range(1000, 20001, 4000):
        for vel in range(50, 201, 50):
            if vel > 50 and lower_alt == upper_alt:
                continue
            if lower_alt > upper_alt:
                continue
            elif lower_alt == upper_alt:
                vel = 0
            print("Running low: ", lower_alt, ", up: ", upper_alt, ", vel: ", vel)
            ctrl = Controller('500m')
            ctrl.run_data_gathering(lower_alt, upper_alt, vel)

'''
