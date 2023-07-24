from motor_test import test_motor
import time
from pymavlink import mavutil
import socket
 
'''sets up socket Host'''
HOST = "0.0.0.0"
'''sets up the port'''
Port = 8000
socket.socket(socket.AF_INET, socket.SOCK_STREAM)
'''with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, Port))
    s.listen(1)
    conn, addr = s.accept()
    with conn:
        print ('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data: break
            conn.sendall(data)
            '''
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
        # time.sleep(0.2)
        step += 0.2

    

if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    def forwardlinesegment(t, m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[m, m, -m, -m, 0, 0])
         run_motors_timed(mav_connection, seconds=t/2, motor_settings=[-m/4, -m/4, m/4, m/4, 0, 0])
    def leftturn(t,m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[-m, m, m, -m, 0, 0])
    def rightturn(t, m):
         run_motors_timed(mav_connection, seconds=t, motor_settings=[m, -m, -m, m, 0, 0])
    def rightstrafe(t, m):
         run_motors_timed(mav_connection, seconds = t, motor_settings= [m, -m, m, -m])
    def leftstrafe(t, m):
         run_motors_timed(mav_connection, seconds=t, motor_settings= [ -m, m, -m, m])
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, Port))
        s.listen(1)
        print("got here")
        conn, addr = s.accept()
        print(HOST)
        with conn:
            print("Address: ", addr)
            while True:

                data = conn.recv(1024)
                data = data.decode('UTF-8')
                if not data: break
               
                if data.strip() == "forward":
                    print("data received")
                    run_motors_timed(mav_connection, seconds=10, motor_settings=[-100, -100, 100, 100, 0, 0])
                    arm_rov(mav_connection)
                if data.strip() == "right turn":
                    print("data received")
                    rightturn(5, 100)
                    arm_rov(mav_connection)
                if data.strip() == "left turn":
                    print("data received")
                    leftturn(5, 100)
                    arm_rov(mav_connection)
                if data.strip() == "left":
                    print("data received")
                    leftstrafe(10, 100)
                    arm_rov(mav_connection)
                if data.strip() == "right":
                    print("data received")
                    rightstrafe(10, 100)
                    arm_rov(mav_connection)
                else:
                    continue
    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
   
    def draw_Mit(S):
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
       
    

    

    ####
    # Disarm ROV and exit
    ####
   

