import mc_control
import mc_solver
import mc_rbdyn
import mc_rtc
import mc_tasks
import eigen
import math
import sva

# Constants (enumerates)
APPROACH = 0
HANDLE = 1
OPEN = 2

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):

        #Add constraints
        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)

        #Add a posture tasks
        self.qpsolver.addTask(self.postureTask)
        
        #Add contacts
        self.addContact(self.robot().name(), "ground", "LeftFoot", "AllGround")
        self.addContact(self.robot().name(), "ground", "RightFoot", "AllGround")

        # Create a CoM task
        self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0) # inputs are, robot model, robot index 0 (main robot), gain (kp), weight (priority)
        self.qpsolver.addTask(self.comTask)

        #Add a end-effector task
        self.efTask = mc_tasks.EndEffectorTask("l_wrist", self.robots(), 0, 10.0, 1000.0)
        self.qpsolver.addTask(self.efTask)

        # Reduce the posture task stiffness
        self.postureTask.stiffness(1)

        # Paramters
        #   for posture task
        self.jointName = ("NECK_Y").encode('utf-8')
        self.jointIndex = self.robot().jointIndexByName(self.jointName)
        self.goingLeft = True
        #   for CoM task
        self.comDown = True
        self.comZero = eigen.Vector3d.Zero()
        #   for door openning task
        self.phase = APPROACH


    # Call this in the run callback
    def run_callback(self):
        self.switch_phase()
        return True
    
    # Reset callback
    def reset_callback(self, data):
        # Reset CoM task
        self.comTask.reset() #reset to a default pose
        self.comZero = self.comTask.com()

        # Reset end-effector task
        self.efTask.reset()

        #Reset the door
        self.robots().robot(1).posW(sva.PTransformd(sva.RotZ(math.pi), eigen.Vector3d(0.7, 0.5, 0)))
        self.doorKinematics = mc_solver.KinematicsConstraint(self.robots(), 1, self.qpsolver.timeStep)
        self.qpsolver.addConstraintSet(self.doorKinematics)
        self.doorPosture = mc_tasks.PostureTask(self.qpsolver, 1, 5.0, 1000.0)
        self.qpsolver.addTask(self.doorPosture)

        # Reach the handle task
        self.handTask = mc_tasks.SurfaceTransformTask("RightGripper", self.robots(), 0, 5.0, 1000.0)
        self.qpsolver.addTask(self.handTask)
        # Set a target relative to the handle position
        self.handTask.target(sva.PTransformd(eigen.Vector3d(0, 0, -0.025))* self.robots().robot(1).surfacePose("Handle"))

    # New method for our controller
    def switch_phase(self):
        if (self.phase == APPROACH and self.handTask.eval().norm() < 0.05 and self.handTask.speed().norm() < 1e-4):
            # Add a new contact
            self.addContact(self.robot().name(), "door", "RightGripper", "Handle")
            # Remove the surface transform task
            self.qpsolver.removeTask(self.handTask)
            # Keep the robot in its current posture
            self.postureTask.reset()
            self.comTask.reset()
            # Target new handle position
            self.doorPosture.target({"handle".encode('utf-8'): {-1.0}})
            # Switch phase
            self.phase = HANDLE
        elif self.phase == HANDLE and self.doorPosture.eval().norm() < 0.01:
            # Update door opening target
            self.doorPosture.target({"door".encode('utf-8'): [0.5]})
            # Switch phase
            self.phase = OPEN

    def switch_target(self):
        if self.goingLeft:
            self.postureTask.target({self.jointName: self.robot().qu[self.jointIndex]})
        else:
            self.postureTask.target({self.jointName: self.robot().ql[self.jointIndex]})
        self.goingLeft = not self.goingLeft

    def switch_com_target(self):
        if self.comDown:
            self.comTask.com(self.comZero - eigen.Vector3d(0, 0, 0.2))
        else:
            self.comTask.com(self.comZero)
        self.comDown = not self.comDown
    
    #load the controller
    @staticmethod
    def create(robot, dt):
        door = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH.decode('utf-8') + "/../mc_int_obj_description", "door")
        ground = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot, door, ground], dt) #the load order decides the index number of each object, the robot is always be 0