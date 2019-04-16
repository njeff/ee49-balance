import gc

class Balance:
    radToDeg = 57.3 # radians to degrees, really just another scaling factor

    def __init__(self, lMotor, rMotor, imu, dt):
        self.pidL = lMotor
        self.pidR = rMotor
        self.imu = imu
        self.dt = dt

        # Working PID Constants
        self.kp = 219
        self.ki = 45

        # the actual setpoint (takes into account position feedback)
        self.setPoint  = 0.07
        # the upward angle if at starting position (no position feedback)
        self.basePoint = 0.07 
        self.balancing = False
        self.count = 0
        # integrator state
        self.integ = 0

    # set PI constants
    def set_balance_pi(self, p, i):
        self.kp = p
        self.ki = i
    
    # for keeping track of how long it has been balancing
    def increment_count(self):
        self.count += 1

    def do_balance(self):
        angle = (self.imu.euler()[2] - 90) / self.radToDeg
        print(angle)
        
        # if relatively straigt up
        if (abs(angle) < 0.1):
            # and has been held up for 3 seconds while not actively balancing
            if (self.count > 3 and not self.balancing):
                # enable balancing
                self.balancing = True
                self.basePoint = angle

        error = angle - self.setPoint
        self.integ += error * self.dt / 0.02
        # clamp
        self.integ = max(-20, min(self.integ, 20))

        # derivative not used
        output = self.kp * error + self.ki * self.integ # + self.kd * rate
        speedSetPoint = max(-40, min(40, output))

        lp = self.pidL.enc.get_count()
        rp = self.pidR.enc.get_count()
        
        # position feedback to not deviate too far from start point
        avgpos = (lp + rp) / 2
        self.setPoint = self.basePoint + avgpos * -0.00012

        # if too steep do not try to recover
        if (abs(angle) > 0.8):
            self.count = 0
            self.balancing = False
        
        if self.balancing:
            # balancing, do drive the motors
            lcps = self.pidL.pi_control(speedSetPoint*50, self.dt*1000, 0.045, 0.5)
            rcps = self.pidR.pi_control(speedSetPoint*50, self.dt*1000, 0.045, 0.5)
        else:
            # not balancing; clear out position counts and integrator
            # clear_count is the new method for final lab
            self.pidL.enc.clear_count()
            self.pidR.enc.clear_count()
            self.integ = 0

            # stop motors
            lcps = self.pidL.pi_control(0, self.dt*1000, 0.04, 0.5)
            rcps = self.pidR.pi_control(0, self.dt*1000, 0.04, 0.5)

        gc.collect()
