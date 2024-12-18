import mc_control
import mc_rbdyn
import mc_rtc
import mc_tasks
import eigen
import math
import sva

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

    # Controller call back for each control cycle, here we gitve targets to the tasks
    def run_callback(self):
        if (
            abs(
                self.postureTask.posture()[self.jointIndex][0]
                - self.robot().mbc.q[self.jointIndex][0]
            )
            < 0.05
        ):
            self.switch_target()
        
        if self.comTask.eval().norm()<0.01:
            self.switch_com_target()

        # Get the current objective
        pt = self.efTask.get_ef_pose()
        # Update the rotation and position objective
        self.efTask.set_ef_pose(sva.PTransformd(sva.RotY(-math.pi / 2), eigen.Vector3d(0.5, -0.5, 1.2))
)
    
        return True
    
    # Reset callback
    def reset_callback(self, data):
        # Reset CoM task
        self.comTask.reset() #reset to a default pose
        self.comZero = self.comTask.com()

        # Reset end-effector task
        self.efTask.reset()

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
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot,env], dt)