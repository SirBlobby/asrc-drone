def clamp(value, limit):
    return max(-limit, min(limit, value))


def clamp_unit(value):
    return max(0.0, min(1.0, value))


class PID:
    def __init__(self, gains, output_limit, integral_limit=None,
                 derivative_filter=0.4):
        self.proportional_gain, self.integral_gain, self.derivative_gain = gains
        self.output_limit = output_limit
        self.integral_limit = (output_limit if integral_limit is None
                               else integral_limit)
        self.derivative_filter = derivative_filter
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def update(self, error, dt, weight=1.0):
        weight = clamp_unit(weight)
        self.integral = clamp(self.integral + weight * error * dt,
                              self.integral_limit)

        previous = error if self.previous_error is None else self.previous_error
        measured_rate = (error - previous) / dt
        self.derivative += self.derivative_filter * (measured_rate -
                                                     self.derivative)
        self.previous_error = error

        output = (self.proportional_gain * error +
                  self.integral_gain * self.integral +
                  self.derivative_gain * self.derivative)
        return clamp(output, self.output_limit)
