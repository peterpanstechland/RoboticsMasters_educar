from time import sleep, monotonic_ns
import board
import pulseio

class RoboHatEduCar(object):
    '''Class for controlling an educational car with stand alone RoboHat MM1 board.

    '''

    # configurations

    DRIVER_PWM_FREQ = 400
    '''pwm frequency of the motor driver'''

    DUTY_CYCLE_MOTOR_MIN = 15000
    '''the lowest duty cycle needed for the motor to keep turning the wheel '''

    DUTY_CYCLE_MOTOR_BRAKING = 7000
    '''the duty cycle needed against the direction of rotation for the motor to brake '''

    DUTY_CYCLE_MOTOR_START = 25000
    '''the duty_cycle to be used for the motor to turn the wheel from rest'''

    DEFAULT_PEDAL_VALUE = 30
    '''default speed in percent 0% - 100%'''

    CTRL_P_COEFF = 400
    ''' P coefficient '''

    ENC_DISTANCE_TO_360DEGREES = 77
    '''wheel distance in encoder ticks for a 360 degree turn'''

    ENC_DISTANCE_TO_1METER = 188
    ''' wheel distance in encoder ticks for the car to travel 1 meter'''

    ENC_DISTANCE_TO_TARGET_RAMPDOWN = 20
    '''when the enc_distance to target is lower than this setting we enter ST_RAMPDOWN state'''

    ENC_DISTANCE_TO_TARGET_BRAKING = 5  # 7
    '''when the enc_distance to target is lower than this setting we enter ST_BRAKING state'''

    CTR_EXIT_RAPDOWN = 200
    '''max amount of cycles we are allowed to stay in ST_RAMPDOWN state before giving up'''

    CTR_EXIT_BRAKING = 20
    '''max amount of cycles we are allowed to stay in ST_BRAKING state before giving up'''

    # class constants
    DIR_BACKWARD = -1
    DIR_FORWARD = 1

    CMD_TURN_RIGHT = -2
    '''Command to turn right'''

    CMD_DRV_BACKWARD = -1
    '''Command to drive backward'''

    CMD_STOP = 0
    '''Command to stop'''

    CMD_DRV_FORWARD = 1
    '''Command to drive forward'''

    CMD_TURN_LEFT = 2
    '''Command to turn left'''

    ST_STARTING = 11
    ST_CRUISING = 12
    ST_RAMPDOWN = 13
    ST_BRAKING = 14
    ST_TARGET_REACHED = 15

    WHEEL_LEFT = 0
    WHEEL_RIGHT = 1

    def __init__(self):
        # Left Ain1(backward) Ain2(Forward) Right Bin1(backward) Bin2(Forward)
        # Ain2
        self.driver_left_in2 = pulseio.PWMOut(board.SERVO3, frequency=RoboHatEduCar.DRIVER_PWM_FREQ)
        # Ain1
        self.driver_left_in1 = pulseio.PWMOut(board.SERVO4, frequency=RoboHatEduCar.DRIVER_PWM_FREQ)
        # Bin1
        self.driver_right_in1 = pulseio.PWMOut(board.SERVO5, frequency=RoboHatEduCar.DRIVER_PWM_FREQ)
        # Bin2
        self.driver_right_in2 = pulseio.PWMOut(board.SERVO6, frequency=RoboHatEduCar.DRIVER_PWM_FREQ)

        self.wheel_enc_left = pulseio.PulseIn(board.SERVO1, maxlen=30)
        self.wheel_enc_right = pulseio.PulseIn(board.SERVO2, maxlen=30)
        self.wheel_pos_left = 0
        self.wheel_pos_right = 0
        self.direction_left_wheel = RoboHatEduCar.DIR_FORWARD
        self.direction_right_wheel = RoboHatEduCar.DIR_FORWARD
        self.ctr_rampdown = [0, 0]
        self.ctr_braking = [0, 0]
        self.stop()

        """Drive the car forward or backward by the specified distance.

        Argument:
        command -- RoboHatEduCar.CMD_DRV_FORWARD or RoboHatEduCar.CMD_DRV_BACKWARD
        distance_in_m -- a number (integer or float) in meters
        """
    def drive(self, command, distance_in_m):
        enc_distance = int(distance_in_m * RoboHatEduCar.ENC_DISTANCE_TO_1METER)
        if enc_distance >= 5:
            self._drive(command, enc_distance)
        else:
            print("Distance too small '{}m'".format(distance_in_m))

        """Turn the car left or right by the specified angle in degrees.

        Argument:
        command -- RoboHatEduCar.CMD_TURN_LEFT or RoboHatEduCar.CMD_TURN_RIGHT
        angle -- a number (integer or float) in degrees
        """
    def turn(self, command, angle):
        enc_distance = int(angle * RoboHatEduCar.ENC_DISTANCE_TO_360DEGREES / 360)
        self._drive(command, enc_distance)

        """Stop the car.
        """
    def stop(self):
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_STOP)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_STOP)

    def _get_wheel_positions(self):
        self.wheel_pos_left = self.wheel_pos_left + len(self.wheel_enc_left) * self.direction_left_wheel
        self.wheel_enc_left.clear()

        self.wheel_pos_right = self.wheel_pos_right + len(self.wheel_enc_right) * self.direction_right_wheel
        self.wheel_enc_right.clear()

        return (self.wheel_pos_left, self.wheel_pos_right)

    def _pedal_to_duty_cycle(self, pedal_value):
        if pedal_value <= 0 or pedal_value > 100:
            duty_cycle = 0
        else:
            duty_cycle = int(pedal_value * (65535 - RoboHatEduCar.DUTY_CYCLE_MOTOR_MIN) / 100) + RoboHatEduCar.DUTY_CYCLE_MOTOR_MIN
        return duty_cycle

    def _drive(self, command, enc_distance):
        if (command == RoboHatEduCar.CMD_DRV_FORWARD
            or command == RoboHatEduCar.CMD_DRV_BACKWARD
            or command == RoboHatEduCar.CMD_TURN_LEFT
            or command == RoboHatEduCar.CMD_TURN_RIGHT):
            pass
        else:
            raise ValueError("RoboHatEduCar error, command not defined `{}'.".format(command.__name__))

        distance_reached = False
        self.ctr_rampdown = [0, 0]
        self.ctr_braking = [0, 0]
        start_pos_left, start_pos_right = self._get_wheel_positions()
        duty_cycle_default = self._pedal_to_duty_cycle(RoboHatEduCar.DEFAULT_PEDAL_VALUE)

        print("{} command received with distance: {} start left pos: {}, start right pos: {}".format(
            command, enc_distance, start_pos_left, start_pos_right))

        while distance_reached is False:
            start_time = monotonic_ns()
            pos_left, pos_right = self._get_wheel_positions()
            abs_distance_left = abs(pos_left - start_pos_left)
            abs_distance_right = abs(pos_right - start_pos_right)
            error = abs_distance_right - abs_distance_left

            ctrl_state_left = self._next_state_logic(RoboHatEduCar.WHEEL_LEFT, abs_distance_left, enc_distance)
            ctrl_state_right = self._next_state_logic(RoboHatEduCar.WHEEL_RIGHT, abs_distance_right, enc_distance)

            error_left = error * RoboHatEduCar.CTRL_P_COEFF
            error_right = - error * RoboHatEduCar.CTRL_P_COEFF

            duty_cycle_left = self._output_logic(RoboHatEduCar.WHEEL_LEFT, ctrl_state_left, command, duty_cycle_default, error_left)
            duty_cycle_right = self._output_logic(RoboHatEduCar.WHEEL_RIGHT, ctrl_state_right, command, duty_cycle_default, error_right)

            if  (ctrl_state_left == RoboHatEduCar.ST_TARGET_REACHED and
                 ctrl_state_right == RoboHatEduCar.ST_TARGET_REACHED):
                distance_reached = True

            duration = monotonic_ns() - start_time
            # print("duration: {}".format(duration))
            # print("left pos: {} st: {} dc: {}, right pos: {} st:{} dc:{}".format(
            #    pos_left, ctrl_state_left, duty_cycle_left, pos_right, ctrl_state_right, duty_cycle_right))
            duration = monotonic_ns() - start_time
            if duration <= 0.009:
                sleep(0.01 - duration/1e9)

        # print("left pos: {}, right pos: {}".format(pos_left, pos_right))

    def _next_state_logic(self, wheel, curr_distance, target_distance):
        distance_to_target = target_distance - curr_distance
        if(distance_to_target <= 0):
            state = RoboHatEduCar.ST_TARGET_REACHED
        elif(curr_distance <= 0):
            state = RoboHatEduCar.ST_STARTING
        elif(distance_to_target > RoboHatEduCar.ENC_DISTANCE_TO_TARGET_RAMPDOWN):
            state = RoboHatEduCar.ST_CRUISING
        elif(distance_to_target > RoboHatEduCar.ENC_DISTANCE_TO_TARGET_BRAKING):
            if self.ctr_rampdown[wheel] < RoboHatEduCar.CTR_EXIT_RAPDOWN:
                state = RoboHatEduCar.ST_RAMPDOWN
                self.ctr_rampdown[wheel] += 1
            else:
                state = RoboHatEduCar.ST_TARGET_REACHED
        else:
            if self.ctr_braking[wheel] < RoboHatEduCar.CTR_EXIT_BRAKING:
                state = RoboHatEduCar.ST_BRAKING
                self.ctr_braking[wheel] += 1
            else:
                state = RoboHatEduCar.ST_TARGET_REACHED
        return state

    def _output_logic(self, wheel, ctrl_state, command, duty_cycle_cruising, error):
        braking = False
        if(ctrl_state == RoboHatEduCar.ST_STARTING):
            duty_cycle = RoboHatEduCar.DUTY_CYCLE_MOTOR_START
        elif(ctrl_state == RoboHatEduCar.ST_RAMPDOWN):
            duty_cycle = RoboHatEduCar.DUTY_CYCLE_MOTOR_MIN + error
        elif(ctrl_state == RoboHatEduCar.ST_BRAKING):
            command = -command  # reverse
            braking = True
            duty_cycle = RoboHatEduCar.DUTY_CYCLE_MOTOR_BRAKING
        elif(ctrl_state == RoboHatEduCar.ST_TARGET_REACHED):
            duty_cycle = 0
            command = RoboHatEduCar.CMD_STOP
        elif(ctrl_state == RoboHatEduCar.ST_CRUISING):
            duty_cycle = duty_cycle_cruising + error
        else:
            raise ValueError("RoboHatEduCar error, state not defined `{}'.".format(ctrl_state))
        self._drv_wheel(wheel, command, duty_cycle=duty_cycle, braking=braking)
        return duty_cycle

    def _drv_wheel(self, wheel, command, duty_cycle=None, pedal_value=DEFAULT_PEDAL_VALUE, braking=False):
        if duty_cycle is None:
            duty_cycle = self._pedal_to_duty_cycle(pedal_value)

        if(wheel == RoboHatEduCar.WHEEL_LEFT):
            driver_in1 = self.driver_left_in1
            driver_in2 = self.driver_left_in2
            direction_attr_name = 'direction_left_wheel'
        elif(wheel == RoboHatEduCar.WHEEL_RIGHT):
            driver_in1 = self.driver_right_in1
            driver_in2 = self.driver_right_in2
            direction_attr_name = 'direction_right_wheel'
        else:
            raise ValueError("RoboHatEduCar error, wheel not defined `{}'.".format(wheel))

        if  (command == RoboHatEduCar.CMD_DRV_FORWARD
             or (wheel == RoboHatEduCar.WHEEL_LEFT and command == RoboHatEduCar.CMD_TURN_RIGHT)
             or (wheel == RoboHatEduCar.WHEEL_RIGHT and command == RoboHatEduCar.CMD_TURN_LEFT)):
            driver_in1.duty_cycle = 0
            driver_in2.duty_cycle = duty_cycle
            if not braking:
                setattr(self, direction_attr_name, RoboHatEduCar.DIR_FORWARD)
        elif (command == RoboHatEduCar.CMD_DRV_BACKWARD
              or (wheel == RoboHatEduCar.WHEEL_LEFT and command == RoboHatEduCar.CMD_TURN_LEFT)
              or (wheel == RoboHatEduCar.WHEEL_RIGHT and command == RoboHatEduCar.CMD_TURN_RIGHT)):
            driver_in1.duty_cycle = duty_cycle
            driver_in2.duty_cycle = 0
            if not braking:
                setattr(self, direction_attr_name, RoboHatEduCar.DIR_BACKWARD)
        elif command == RoboHatEduCar.CMD_STOP:
            driver_in1.duty_cycle = 0
            driver_in2.duty_cycle = 0
        else:
            raise ValueError("RoboHatEduCar error, command not defined `{}'.".format(command))

    """Drive the car according to a test pattern.
    """
    def drv_testPattern(self):
        print("Test pattern")
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_FORWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_STOP)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_FORWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_STOP)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_BACKWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_STOP)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_BACKWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_STOP)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_BACKWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_FORWARD)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_FORWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_BACKWARD)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_FORWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_FORWARD)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_DRV_BACKWARD)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_DRV_BACKWARD)
        sleep(1)
        self._drv_wheel(RoboHatEduCar.WHEEL_LEFT, RoboHatEduCar.CMD_STOP)
        self._drv_wheel(RoboHatEduCar.WHEEL_RIGHT, RoboHatEduCar.CMD_STOP)


class EduCarTurtle(object):
    def __init__(self, car):
        self.car = car

    # def position():

    def forward(self, distance):
        """Move the turtle forward by the specified distance.

        Aliases: forward | fd

        Argument:
        distance -- a number (integer or float)

        Move the turtle forward by the specified distance, in the direction
        the turtle is headed.
        """
        self.car.drive(RoboHatEduCar.CMD_DRV_FORWARD, distance)

    def back(self, distance):
        """Move the turtle backward by distance.
        Aliases: back | backward | bk
        Argument:
        distance -- a number
        Move the turtle backward by distance ,opposite to the direction the
        turtle is headed. Do not change the turtle's heading.
        """
        self.car.drive(RoboHatEduCar.CMD_DRV_BACKWARD, distance)

    def left(self, angle):
        """Turn turtle left by angle units.

        Aliases: left | lt

        Argument:
        angle -- a number (integer or float)

        Turn turtle left by angle degrees.
        """
        self.car.turn(RoboHatEduCar.CMD_TURN_LEFT, angle=angle)

    def right(self, angle):
        """Turn turtle right by angle units.

        Aliases: right | rt

        Argument:
        angle -- a number (integer or float)

        Turn turtle right by angle degrees.
        """
        self.car.turn(RoboHatEduCar.CMD_TURN_RIGHT, angle=angle)

    # method aliases
    fd = forward
    bk = back
    backward = back
    rt = right
    lt = left
