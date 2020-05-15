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
        W = Settings.G * m * Settings.M / (Settings.R + y_t) ** 2
        # compute acceleration ma = (T - D - W)
        if v_t > 0:
            a_t = 1/m * (T - D - W)
        else:
            a_t = 1/m * (T + D - W)
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
            cost += np.abs(target_state[0] - state[0])
            # 0.065
            cost += np.abs(target_state[1] - state[1])
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

    def model_validation(self, start, end, sim_time):
        data = []
        throttle_values = np.linspace(start, end, sim_time / self.dt)
        for throttle in throttle_values:
            actual_state = self.vessel.get_status()
            # get the model prediction
            model_state = self.model([actual_state['Altitude'], actual_state['Vertical Velocity']], throttle, debug=True)
            # set the rockets throttle
            self.vessel.set_throttle(throttle)
            # wait for dt
            time.sleep(self.dt)
            # read actual value
            actual_state = self.vessel.get_status()
            data.append(model_state + [actual_state['Altitude'], actual_state['Vertical Velocity'],
                                       actual_state['Thrust'], actual_state['Drag Z'], self.vessel.get_mass()])
        return data

