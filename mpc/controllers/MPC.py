import numpy as np
from mpc.Settings import Settings
from scipy.optimize import minimize
import time

class MPC:
    def __init__(self, vessel, horizon=5, dt=1):
        self.vessel = vessel
        self.horizon = horizon
        self.available_thrust = self.vessel.get_available_thrust()
        self.dt = dt
        self.drag_model = np.poly1d(np.load('models/drag.npy'))
        self.m = self.vessel.get_mass()

    def model(self, prev_state, throttle, debug=False):
        y_t = prev_state[0]
        v_t = prev_state[1]
        T = self.vessel.get_available_thrust() * throttle
        D = self.drag_model(v_t)
        # compute weight mg
        # m = self.vessel.get_mass()
        m = self.m
        print('mass: ', m)
        W = Settings.G * m * Settings.M / (Settings.R + y_t) ** 2
        # compute acceleration ma = (T - D - W)
        a_t = 1 / m * (T - np.sign(v_t) * D - W)
        # compute the vertical speed
        v_t_1 = v_t + a_t * self.dt
        # compute new altitude
        y_t_1 = y_t + v_t_1 * self.dt
        # return new state
        if debug:
            return [y_t, v_t, y_t_1, v_t_1, T, D, m, W, a_t]
        else:
            return [y_t_1, v_t_1]

    def cost(self, u, init_state, target_state):
        state = init_state
        cost = 0
        for i in range(self.horizon):
            state = self.model(state, u[i])
            # cost += 0.75 * np.abs(target_state[0] - state[0])
            # cost += np.abs(target_state[1] - state[1])

            cost += 0.2 * np.square(target_state[0] - state[0])
            cost += np.square(target_state[1] - state[1])
            cost += np.square(u[i] * 20)
            # print(0.2 * np.square(target_state[0] - state[0]))
            # print(np.square(target_state[1] - state[1]))
            # print(np.square(u[i] * 10))
        return cost

    def get_optimal_throttle(self, current_state, target_state):
        u = np.zeros(self.horizon)
        bounds = [[0, 1] for i in range(self.horizon)]
        optimal_throttle = minimize(self.cost, u, (current_state, target_state),
                                    method='SLSQP',
                                    bounds=bounds,
                                    tol=1e-2,
                                    # options={'maxiter': 1, 'ftol': 1}
                                    )
        return optimal_throttle.x[0]

    def model_validation(self, inputs):
        data = []
        # generate throttle values for each timestamp
        throttle_values = [x for inp in inputs for x in np.linspace(inp[0], inp[1], int(inp[2]/self.dt))]
        # get total simulation time
        sim_time = sum([inp[2] for inp in inputs])
        model_state = []
        for timestamp in range(int(sim_time/self.dt)):
            actual_state = self.vessel.get_status()
            # every time horizon is reached, update simulation input values with actual state
            if self.horizon == 0:
                model_input = [actual_state['Altitude'], actual_state['Vertical Velocity']]
            elif timestamp % self.horizon == 0:
                model_input = [actual_state['Altitude'], actual_state['Vertical Velocity']]
            else:
                model_input = [model_state[2], model_state[3]]
            # get the model prediction
            model_state = self.model(model_input, throttle_values[timestamp], debug=True)
            # set the rockets throttle
            input_throttle = throttle_values[timestamp]
            if actual_state['Altitude'] < 200:
                input_throttle = 1
            self.vessel.set_throttle(input_throttle)
            # wait for dt
            time.sleep(self.dt)
            # read actual value
            actual_state = self.vessel.get_status()
            data.append(model_state + [actual_state['Altitude'], actual_state['Vertical Velocity'],
                                       actual_state['Thrust'], actual_state['Drag Z'], self.vessel.get_mass(),
                                       input_throttle])
        return data

