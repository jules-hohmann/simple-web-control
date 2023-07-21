from motor_test import test_motor
import time
from pymavlink import mavutil


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)
        step += 0.2

    

if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    def forwardlinesegment(t, m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[m, m, -m, -m, 0, 0])
         run_motors_timed(mav_connection, seconds=t/2, motor_settings=[-m/4, -m/4, m/4, m/4, 0, 0])
    def leftturn(t,m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[-m, m, m, -m, 0, 0])
    def rightturn(t, m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[m, -m, -m, m, 0, 0])
    def draw_Mit(S):
        #M
        
        forwardlinesegment(4*S, 100)
        rightturn(3, 100)
        forwardlinesegment(4*S, 100)
        leftturn(3, 100)
        forwardlinesegment(4*S, 100)
        rightturn(3, 100)
        forwardlinesegment(4*S, 100)
        leftturn(2, 80)
        forwardlinesegment(5.5*S, 100)
        forwardlinesegment(2.75*S,-100)
        leftturn(2, 75)
        forwardlinesegment(4*S, 100)
        rightturn(2, 75)
        forwardlinesegment(2.5*S, -100)
        forwardlinesegment(8*S, 100)
        forwardlinesegment(2.5*S, -100)
        rightturn (2, 75)
        forwardlinesegment(4*S, 100)
        #I
        '''
        run_motors_timed(mav_connection, seconds=5, motor_settings=[50, 0, 0, -50, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[50, 50, -50, -50, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[-25, -25, 25, 25, 0, 0])
        run_motors_timed(mav_connection, seconds=5, motor_settings=[100, 0, 0, -100, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[100, 100, -100, -100, 0, 0])
        run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 100, -100, 0, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[-25, 25, -25, -25, 0, 0])
        #last part of I connecting to T
        run_motors_timed(mav_connection, seconds=4, motor_settings=[100, 100, -100, -100, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[-25, 25, -25, -25, 0, 0])
        run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 100, -100, 0, 0, 0])
        run_motors_timed(mav_connection, seconds=2, motor_settings=[100, 100, -100, -100, 0, 0])
    '''
    

    draw_Mit(2)

    ####
    # Disarm ROV and exit
    ####
   

