

def drag_poly_2d(vertical_velocity, altitude):
    return (1.43404918e+02 +
            vertical_velocity * -1.07837652e-01 +
            altitude * 1.58209854e-02 +
            vertical_velocity ** 2 * 4.32141454e-01 +
            vertical_velocity ** 2 * altitude * -4.05546606e-05 +
            vertical_velocity ** 2 * altitude ** 2 * 1.00025150e-09 +
            altitude ** 2 * -1.36906589e-06 +
            vertical_velocity * altitude ** 2 * -1.82363448e-09 +
            vertical_velocity * altitude * 2.98008973e-05)

