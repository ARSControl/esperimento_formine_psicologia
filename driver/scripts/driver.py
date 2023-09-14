#! /usr/bin/env python3

import rospy, rospkg
import os, csv

from dataclasses import dataclass
from typing import List

# Import ROS Messages
from std_msgs.msg import Int8, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

# Import UR_RTDE Controller Services
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest
from ur_rtde_controller.srv import GetInverseKinematic, GetInverseKinematicRequest

@dataclass 
class ShapeObject: #oggetto che contiene numero e posizione in giunti 

    """ Dataclass for a Shape Object """

    number: int
    joint_positions: List[float]

class PosDriver(): #creazione di un'altra classe

    # Initialize Variables
    actual_position = [0.0] * 6

    # Home and Place Positions
    home_position  = [1.2372798919677734, -1.550156907444336, 2.0900400320636194, -2.0843998394408167, -1.5195234457599085, -0.2930181662188929]
    place_position = [2.555208683013916, -1.1432717603496094, 1.9683178106891077, -2.4419647655882777, -1.578958813344137, 2.5797250270843506]
    temp_position = [2.5360817909240723, -2.4209352932372035, 2.562695328389303, -1.7516471348204554, -1.4983561674701136, -0.7107937971698206]

    # Movement Velocity
    velocity = 0.4

    # Trajectory Executed Flag
    trajectory_executed = False

    # New Arduino Message Flag
    new_arduino_message = False

    # Piece Number
    piece_number = None

    # Piece Placed Message
    piece_placed_msg = Int8()
    piece_placed_msg.data = 1

    def __init__(self): 

        """ Class Constructor """

        # Get ROS Package Path
        rospack = rospkg.RosPack() #vado a prendere il package attuare che è il driver in modo da usarlo quando carico il documento (79)
        package_path = rospack.get_path('driver')

        # ROS Publishers
        self.piece_placed_pub = rospy.Publisher('position_place', Int8, queue_size=1)
        self.move_robot_pub = rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command', JointTrajectoryPoint, queue_size=1)

        # ROS Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.JointStateCallback)
        self.arduino_piece_sub = rospy.Subscriber('piece_number', Int8, self.ArduinoCallback)
        self.trajectory_executed_sub = rospy.Subscriber('/ur_rtde/trajectory_executed', Bool, self.TrajectoryExecutedCallback)

        # ROS Service Clients
        self.gripper_client = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)
        self.get_FK_client  = rospy.ServiceProxy('/ur_rtde/getFK', GetForwardKinematic)
        self.get_IK_client  = rospy.ServiceProxy('/ur_rtde/getIK', GetInverseKinematic)

        # Create the Shape Object Array array che contiene tutte le posizioni delle varie formine 
        self.shape_array = []

        # Load Position Data from CSV File
        self.load_position_from_csv(os.path.join(package_path, 'doc/hardcoded_position.csv')) #collego i due percorsi package e il documento csv

        rospy.sleep(1)

        # Move to Home Position
        self.moveRobot(self.home_position)
        print('Reached Home Position - Ready to Start')

    def load_position_from_csv(self, filename: str): #caricare le oisizioni dal csv

        """ Load Position from CSV File """

        # Open the CSV File in Read Mode
        with open(filename, 'r') as csv_file:

            # Create a CSV Reader Object
            csv_reader = csv.reader(csv_file)

            # Loop through the Rows in the CSV File
            for row in csv_reader: 

                # Create a new Shape Object with the Extracted Object Number and Positions
                new_shape_object = ShapeObject(int(row[0]), [float(x) for x in row[1:]])   

                # Append the Shape Object to the Array
                self.shape_array.append(new_shape_object)   #aggiungo tutto all'array 

    def JointStateCallback(self, data: JointState): 

        """ Joint State Subscriber Callback """

        # Save Actual Join Position
        self.actual_position = data.position

    def ArduinoCallback(self, data: Int8):

        """ Arduino Subscriber Callback """

        # Get the Piece Number
        self.piece_number = data.data
        self.new_arduino_message = True

        # Log the Piece Number
        rospy.loginfo(f'Received Piece Number: {self.piece_number}')

    def TrajectoryExecutedCallback(self, data: Bool):

        """ Trajectory Executed Subscriber Callback """

        # Get Trajectory Executed Message
        self.trajectory_executed = data.data

    def moveRobot(self, position: List[float]):

        """ Move Robot to Position """

        # Create Message
        msg = JointTrajectoryPoint()
        msg.positions = position
        msg.velocities = [self.velocity] * 6

        # Publish Message
        self.move_robot_pub.publish(msg)

        # Wait for Trajectory to be Executed
        while not self.trajectory_executed and not rospy.is_shutdown(): #muovo il robot e aspetto finchè bool non diventa true e quindi finchè il robot non è arrivato
            rospy.loginfo_throttle(10,'Wait for Trajectory Execution')

        # Reset Trajectory Executed Flag
        print('Trajectory Executed')
        self.trajectory_executed = False

    def moveGripper(self, position: float, speed: float, force: float, wait: float = 1.0):

        """ Move Gripper to Position """

        # Service Request Creation
        request = RobotiQGripperControlRequest()

        # Service Filling
        request.position = position
        request.speed = speed
        request.force = force

        # Wait For Service
        rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')

        # Call Service
        self.gripper_client(request)

        # Wait for Gripper Movement
        rospy.sleep(wait)  

    def compute_pick_position_offset(self, pick_position: List[float], offset: float = 0.1): 

        """ Compute Pick Position Offset """

        # Forward Kinematics Service Request Creation
        fk_request = GetForwardKinematicRequest()
        fk_request.joint_position = pick_position

        # Wait and Call FK Service
        rospy.wait_for_service('/ur_rtde/getFK')
        fk_response = self.get_FK_client(fk_request)

        # Check if FK Service Call was Successful
        if fk_response.success: tcp_position: Pose = fk_response.tcp_position
        else: raise Exception('FK Service Call Failed')

        # Add Z-Offset
        tcp_position.position.z += offset

        # Inverse Kinematics Service Request Creation
        ik_request = GetInverseKinematicRequest()
        ik_request.tcp_position = tcp_position
        ik_request.near_position = pick_position

        # Wait and Call IK Service
        rospy.wait_for_service('/ur_rtde/getIK')
        ik_response = self.get_IK_client(ik_request)

        # Check if FK Service Call was Successful
        if ik_response.success: joint_position: List[float] = ik_response.joint_position
        else: raise Exception('IK Service Call Failed')

        return joint_position

    def main_spinner(self):

        """ Main Spinner Function """

        # Check if a New Arduino Message has been Received
        if self.new_arduino_message == 1:

            # Get Requested Object Position
            pick_position = self.shape_array[self.piece_number - 1].joint_positions

            # Assert Received Piece Number == Selected Piece Number
            assert self.shape_array[self.piece_number - 1].number == self.piece_number, 'Piece Number Mismatch'

            # Compute Pick and Place Position + 10cm Offset
            pick_position_offset = self.compute_pick_position_offset(pick_position, 0.1)
            place_position_offset = self.compute_pick_position_offset(self.place_position, 0.1)

            # Phase 0: Move Robot to Pick Position + 10cm Offset
            self.moveRobot(pick_position_offset)

            # Open Gripper
            self.moveGripper(100, 100, 100, 0.5)

            # Phase 1: Move Robot to Pick Position
            self.moveRobot(pick_position)

            # Close Gripper
            self.moveGripper(0, 100, 100, 1.0)

            # Phase 2: Move Robot to Pick Position + 10cm Offset
            self.moveRobot(pick_position_offset)

            # Phase 2.5: Move Robot to Temp Position + 10cm Offset
            self.moveRobot(self.temp_position)

            # Phase 3: Move Robot to Place Position + 10cm Offset
            self.moveRobot(place_position_offset)

            # Phase 4: Move Robot to Place Position
            self.moveRobot(self.place_position)

            # Open Gripper
            self.moveGripper(100, 100, 100, 1.0)

            # Reset Arduino Message Flag
            self.new_arduino_message = 0

            # Phase 5: Move Robot to Place Position + 10cm Offset
            self.moveRobot(place_position_offset)

            # Phase 5.5: Move Robot to Temp Position + 10cm Offset
            self.moveRobot(self.temp_position)

            # Phase 6: Move Robot to Home Position
            self.moveRobot(self.home_position)

            # Publish that the Piece has been Placed
            self.piece_placed_pub.publish(self.piece_placed_msg)
            rospy.loginfo(f'Piece {self.piece_number} Placed')

if __name__ == '__main__':

    # ROS Initialization
    rospy.init_node('drivejoint', anonymous=True)

    # ROS Rate Initialization
    ros_rate = rospy.Rate(10)

    # Initialize Class
    pos_driver = PosDriver()

    # Main Spinner Function
    while not rospy.is_shutdown(): 

        pos_driver.main_spinner()
        ros_rate.sleep()
