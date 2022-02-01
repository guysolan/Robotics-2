#!/usr/bin/python

"""
kinematics.py

DE3 Applied Robotics, Coursework 1 - MODEL

"""

from math import cos, sin, acos, asin, atan2, sqrt, pi
from numpy.linalg import matrix_rank as rank
from numpy.linalg import pinv, norm
from std_msgs.msg import Float64
from numpy import genfromtxt
import numpy as np
import rospy
import sys


def main(task):
    #GENERATE ROBOT MODEL
    l1 = 1.
    l2 = 1.
    l3 = 1.

    Robot = RobotKineClass([l1,l2,l3])

    # FORWARD KINEMATICS
    if task == "fk":
        print("============================================================")
        print("Testing Forward Kinematics")
        print("------------------------------------------------------------")
        fk_points_file = "test_points/fk_points.csv"
        fk_points = ReadCSV(fk_points_file)
        for i, points in zip(range(len(fk_points)), fk_points):
            print("Test point:\t", i+1)
            joint_space_points = points[:3]
            task_space_points = points[3:]
            print("Joint space points to test:\n", joint_space_points)
            predicted_task_space_points = Robot.getFK(joint_space_points)
            print("Calculated task space points:\n", np.round(predicted_task_space_points, 3))
            error = np.linalg.norm(task_space_points - predicted_task_space_points)
            print("Passed?\t", error<1e-3)
            if error > 1e-3:
                print("Forward Kinematics calculations incorrect, exiting.")
                return
            print("------------------------------------------------------------")
        print("Forward Kinematics calculations correct, well done!")
    elif task == "ws":
        print("============================================================")
        print("Testing Workspace")
        print("------------------------------------------------------------")
        ws_points_file = "test_points/workspace_points.csv"
        ws_points = ReadCSV(ws_points_file)
        for i, points in zip(range(len(ws_points)), ws_points):
            print("Test point:\t", i+1)
            task_space_points = points[:3]
            workspace_flag = points[3:]
            print("Task space points to test:\n", task_space_points)
            predicted_workspace_flag = Robot.checkInWS(task_space_points)
            print("Calculated work space flag:\t", predicted_workspace_flag)
            print("Passed?\t", workspace_flag == predicted_workspace_flag)
            if workspace_flag != predicted_workspace_flag:
                print("Workspace calculations incorrect, exiting.")
                return
            print("------------------------------------------------------------")
        print("Workspace calculations correct, well done!")
    elif task=="ik":
        print("============================================================")
        print("Testing Inverse Kinematics")
        print("------------------------------------------------------------")
        ik_points_file = "test_points/ik_points.csv"
        ik_points = ReadCSV(ik_points_file)
        for i, points in zip(range(len(ik_points)), ik_points):
            print("Test point:\t", i+1)
            task_space_points = points[:3]
            joint_space_points_1 = points[3:6]
            joint_space_points_2 = points[6:]
            print("Task space points to test:\n", task_space_points)
            predicted_joint_space_points_1 = Robot.getIK(task_space_points)[0][0]
            predicted_joint_space_points_2 = Robot.getIK(task_space_points)[0][1]
            print("Calculated joint space points 1:\n", predicted_joint_space_points_1)
            print("Calculated joint space points 2:\n", predicted_joint_space_points_2)
            error1 = np.linalg.norm(joint_space_points_1 - predicted_joint_space_points_1)
            error2 = np.linalg.norm(joint_space_points_2 - predicted_joint_space_points_2)
            pass1 = error1 < 1e-2 and error2 < 1e-2
            error3 = np.linalg.norm(joint_space_points_1 - predicted_joint_space_points_2)
            error4 = np.linalg.norm(joint_space_points_2 - predicted_joint_space_points_1)
            pass2 = error3 < 1e-2 and error4 < 1e-2
            passed = pass1 or pass2
            print("Passed?\t", passed)
            if not passed:
                print("Inverse Kinematics calculations incorrect, exiting.")
                return
            print("------------------------------------------------------------")
        print("Inverse Kinematics calculations correct, well done!")
    elif task=="dk":
        print("============================================================")
        print("Testing Differential Kinematics")
        print("------------------------------------------------------------")
        dk_points_file = "test_points/dk_points.csv"
        dk_points = ReadCSV(dk_points_file)
        for i, points in zip(range(len(dk_points)), dk_points):
            print("Test point:\t", i+1)
            joint_space_points = points[:3]
            joint_space_velocities = points[3:6]
            task_space_velocities = points[6:]
            print("Joint space points to test:\n", joint_space_points)
            print("Joint space velocities to test:\n", joint_space_velocities)
            predicted_task_space_velocities = Robot.getDK(joint_space_points, joint_space_velocities)
            print("Calculated task space velocities:\n", predicted_task_space_velocities)
            error = np.linalg.norm(task_space_velocities - predicted_task_space_velocities)
            passed = error < 1e-3
            print("Passed?\t", passed)
            if not passed:
                print("Differential Kinematics calculations incorrect, exiting.")
                return
            print("------------------------------------------------------------")
        print("Differential Kinematics calculations correct, well done!")
    elif task=="full":
        print("============================================================")
        print("Testing Full Setup")
        print("------------------------------------------------------------")
        #LOAD TASK SPACE POINTS
        points_file = "test_points/points.csv"

        CartPoints = ReadCSV(points_file)

        m = CartPoints.shape[0]

        q_old = np.zeros(3) #initial configuration
        Robot.sendCommands(q_old)
        for i in range(m):

            P = CartPoints[i, :]
            print(" Step " + str(i+1) + "/" + str(m))
            print("Desired Point ", P)
            [q_IK, Positions] = Robot.getIK(P)

            #check solution closer to previous step
            q = Robot.chooseSol(q_IK, q_old)[0]

            #Interpolate
            N_steps = 100
            Q = LinInterp(q_old, q, N_steps)
            q_old = q

            for j in range(N_steps):
                q_comm = Q[j, :]

                #send commands
                Robot.sendCommands(q_comm)

            P_achieved = Robot.getFK(q)
            P_error = P-P_achieved
            print("p achieved", np.round(P_achieved, 3))
            print("position error", np.round(P_error, 3))
            print("------------------------------------------------------------")


#Creates joint publishers
def set_joint_publisher():
    rospy.init_node("joint_positions_node")

    wait = True
    while(wait):
        now = rospy.Time.now()
        if now.to_sec() > 0:
            wait = False

    pubs = []

    for i in range(3):
        topic_name = "/DESE3R/joint_"+str(i)+"_position_controller/command"
        pub = rospy.Publisher(topic_name,Float64,queue_size=1000)
        pubs.append(pub)

    return pubs

#Returns matrix in R^mx3
def ReadCSV(filename):

    Data = genfromtxt(filename, delimiter=',')
    return Data


#returns a matrix of interpolated pints in R^N_stepsx3
def LinInterp(qi,qf,N_steps):

    n = np.linspace(0,1,num=N_steps)

    dq = qf-qi

    Q_interp = np.multiply(dq.reshape(-1,1),n)+qi.reshape(-1,1)
    Q_interp = Q_interp.T

    return Q_interp


#DH_params = parameters for link i
#d,theta,a,alpha
def DH_matrix(DH_params):

    #DH_Params is a variable created within the getFK method fo the RobotKineClass
    #The variable is a copy of the i th row of the DH_Table
    #Indexing this variable extracts a singular value for each D-H variable
    

    d = DH_params[0]
    theta = DH_params[1]
    a = DH_params[2]
    alpha = DH_params[3]
    
    ################################################ TASK 2
    DH_matrix = np.array([[cos(theta), -sin(theta), 0., a],
            [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
            [0., 0., 0., 1.]])
    
    return DH_matrix

class RobotKineClass():

    def __init__(self,link_lengths):

        self.ROSPublishers = set_joint_publisher()

        self.nj = 3    #number of joints
        self.links = link_lengths    # length of links

        ################################################ TASK 1
        #Define DH table for each link. DH_tab in R^njx4
        #d,theta,a,alpha
        
        #Save the DH_table as a numpy array.
        #Link length is accessed by indexing the instance variable 
        self.DH_tab = np.array([[self.links[0],0.,0.,0.],
                                [0.,0.,0.,pi/2],
                                [0,0.,self.links[1],0.],
                                [0., 0., self.links[2], 0.]])
                                
        self.joint_types = 'rrr'	# three revolute joints

    #Computes Forward Kinematics. Returns 3x1 position vector
    def getFK(self,q):

        #Create a 4x4 identity matrix 
        # (this is used within the first iteration of the loop so the first T_i_1_i becomes T_0_i)
        T_0_i_1 = np.identity(4)
        
        #Loop through from joints 0 to n-1
        for i in range(self.nj):
            print(i)

            # Create a new variable from the current row in the D-H table
            DH_params = np.copy(self.DH_tab[i,:])
            #print('q',q)
            #print(DH_params)
            
            
            ### UPDATE THE DH TABLE VALUES ###
            #Check if the joint is a revolute
            if self.joint_types[i] == 'r':
                #If so, add the joint angle to the theta value for the current row
                DH_params[1] = DH_params[1]+q[i]
            #Check if the joint is a prismatic
            elif self.joint_types[i] == 'p':
                #If so, add the current joint angle to the d value for the current row
                DH_params[0] = DH_params[0]+q[i]
            
            # Input the updated table row to get a transformation matrix 
            T_i_1_i = DH_matrix(DH_params) #Pose of joint i wrt i-1
            
            print(T_i_1_i)
            
            ################################################ TASK 3 (replace np.eye(4) with the correct matrices)
            
            # Calculate the transformation matrix up to this point
            T_0_i = np.matmul(T_0_i_1, T_i_1_i) #Pose of joint i wrt base

            # Update the i-1_T_i matrix to be the matrix product up to i
            T_0_i_1 = T_0_i
            
        
        T_0_n_1 = T_0_i
        
        #Create a copy of the last row of the DH Table
        DH_params = np.copy(self.DH_tab[self.nj, :])
        
        #Create the last transformation matrix with the final row of the DH table
        T_n_1_n = DH_matrix(DH_params)
    
        #Multiply the transformation matrix up to n-1 with Tn
        T_0_n = np.matmul(T_0_n_1, T_n_1_n)

        #return the first three rows of the third column
        #these represent the translation part of the transformation matrix
        #this vector is the carteian coordinates of the end effector
        
        return T_0_n[0:3,3]

    #Check if point is in WS. returns true or false
    def checkInWS(self, P): #Define function and set P (the desired point in taskspace) as input
    
        #This step creates varaibles to make key equations easier to write and understand
        xP, yP, zP = P #Seperate P in seperate variables for each dimension
        l0, l1, l2 = self.links #Seperate links in seperate variables for each arm length
        
        ################################################ TASK 4
        val = np.power(xP,2) + np.power(yP,2)+np.power((zP-l0),2) #Calculates distnace of P from the origin of joint 1
        r_max = l1 + l2 #Calcuates the maximum distance the end-effector can be from the origin of joint 1
        r_min = l1 - l2 #Calcuates the minimum distance the end-effector can be from the origin of joint 1

        inWS = True #Sets default output of function to be TRUE

        #Checks to see if point P does not meet the inequalities and is outside the workspace
        if val > r_max**2. or val < r_min**2.: #Checks if val is less then the min distnace or greater than 
                                               #the max distance from joint 1 origin 
            inWS = False #If P is outside workspace chnage output variable to FALSE


        return inWS #Return boolean ouput variable 

    # Solve IK gemoetrically. Returns list of all possible solutions
    def getIK(self,P): #Define function and set P (the desired point in taskspace) as input

        #This step creates varaibles to make key equations easier to write and understand
        xP, yP, zP = P #Seperate P in seperate variables for each dimension
        l0, l1, l2 = self.links #Seperate links in seperate variables for each arm length

        inWS = self.checkInWS(P) #Check if desired point is within the reachable workspace

        q = [] #Create empty joint-space vector
        Poses = [] #Create empty task-space vector

        if not inWS: #If desired pose P not in workspace
            print("OUT OF WS. NO SOLUTION FOUND") #Print "OUT OF WS. NO SOLUTION FOUND" to terminal
            return q,Poses #output empty vectors from function
        
        ################################################ TASK 6
        q_a = np.zeros(3) #Create empty 1x3 array for first joint-space configuration
        q_b = np.zeros(3) #Create empty 1x3 array for second joint-space configuration

        q_a[0] = atan2(yP,xP) #Config. a inverse kinematics equation for the first joint angle
        q_b[0] = q_a[0] #Config. b inverse kinematics equation for the first joint angle

        r = np.sqrt(np.power(xP,2) + np.power(yP,2)) #Define r, the radius of P from the central axis
        z = zP - l0 #Define z, adjusts z dimenions to be in the reference fram of axes 1

        q_a[2] = acos((np.power(r,2)+np.power(z,2)-np.power(l1,2)-np.power(l2,2))/(2*l1*l2)) #Config. a inverse kinematics equation for the third joint angle
        q_b[2] = -q_a[2] #Config. b inverse kinematics equation for the third joint angle. Simply negative of the other config.

        #Both configs. of the second joint angle have the same acos function, with different signs. Below is the acos function.
        q1_acos =  acos((np.power(r,2)+np.power(z,2)+np.power(l1,2)-np.power(l2,2))/(2*l1*np.sqrt(np.power(r,2)+np.power(z,2))))

        q_a[1] = atan2(z,r) - q1_acos #Config. a inverse kinematics equation for the second joint angle
        q_b[1] = atan2(z,r) + q1_acos #Config. b inverse kinematics equation for the second joint angle
        
        q = [q_a, q_b] #bring both configurations into one variable of joint-space
        
        Poses = [self.getFK(q_a), self.getFK(q_b)] #Run both joint-space configurations through forward kinematics to get the pose, this can be compared to P
        return q, Poses #Ouput the cacuated joint-space and the confirmation poses

    #given list of solutions q_IK, returns closest value to old one and list of error norms
    def chooseSol(self,q_IK,q_old):

        diff = []
        q = q_old
        for i in range(len(q_IK)):

            dq = norm(q_IK[i]-q_old)
            diff.append(dq)

        if len(diff) > 0:
            index = np.argmin(diff)
            q = q_IK[index]


        return q,diff
        
    # Computes Differential Kinematics
    def getDK(self, q, q_dot):
        q0, q1, q2 = q
        l1, l2, l3 = self.links
        
        ################################################ TASK 7
        self.Jacobian = np.array([[0., 0., 0.],
                                  [0., 0., 0.],
                                  [0., 0., 0.]])
        x_dot = np.matmul(self.Jacobian, q_dot)
        return x_dot

    #send commands to Gazebo
    def sendCommands(self,q):

        #print("SENDING JOINT VALUES ", q)
        rate = rospy.Rate(100) #Hz
        for i in range(3):

            n_conn = 0
            while not n_conn:
                self.ROSPublishers[i].publish(q[i])
                n_conn = self.ROSPublishers[i].get_num_connections()
                rate.sleep()



if __name__ == "__main__":
    tasks = ['fk', 'ws', 'ik', 'dk', 'full']
    if len(sys.argv) <= 1:
        print('Please include a task to run from the following options:\n', tasks)
    else:
        task = str(sys.argv[1])
        if task in tasks:
            print("Running Coursework 1 -", task)
            main(task)
        else:
            print('Please include a task to run from the following options:\n', tasks)
