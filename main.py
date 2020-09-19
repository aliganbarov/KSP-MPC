from mpc.Controller import Controller
import sys, getopt

# parse arguments, argv[1] - mode, argv[2] - file to load
if len(sys.argv) == 3:
    mode = sys.argv[1]
    load_file = sys.argv[2]
elif len(sys.argv) == 2:
    mode = sys.argv[1]
    load_file = None
else:
    mode = None
    load_file = None

ctrl = Controller(load_file)
ctrl.run_landing(mode)
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
