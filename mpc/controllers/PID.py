

class PID:

    def __init__(self, k_c, k_p, k_i, k_d, init_val, target_val):
        self.k_c = k_c
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.prev_errors = []
        self.prev_val = init_val
        self.target_val = target_val

    def get_val(self, curr_val, print_vals=False):
        # PID components
        proportion = curr_val - self.target_val
        integral = sum(self.prev_errors) / 10.
        differential = (curr_val - self.prev_val)

        # adjust value
        new_val = self.k_c + self.k_p * proportion + self.k_i * integral + self.k_d * differential

        if print_vals:
            print("Error: ", round(curr_val - self.target_val, 2))
            print("P: ", round(proportion, 2))
            print("I: ", round(integral, 2))
            print("D: ", round(differential, 2))
            print("New Val: ", round(new_val, 2))

        # Keep pushing / pulling the integral buffer
        self.prev_errors.append(curr_val - self.target_val)
        if len(self.prev_errors) > 10:
            self.prev_errors = self.prev_errors[1:]

        self.prev_val = curr_val
        return new_val