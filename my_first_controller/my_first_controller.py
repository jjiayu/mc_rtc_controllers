import mc_control
import mc_rbdyn
import mc_rtc
import mc_tasks #Import the mc_tasks module
import eigen

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):

        #Add contacts
        self.addContact(self.robot().name(), "ground", "LeftFoot", "AllGround")
        self.addContact(self.robot().name(), "ground", "RightFoot", "AllGround")

        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)

        # Create a posture task
        self.qpsolver.addTask(self.postureTask)
        self.jointName = ("NECK_Y").encode('utf-8')
        self.jointIndex = self.robot().jointIndexByName(self.jointName)
        self.goingLeft = True
    
        # Create a CoM task
        self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0) # inputs are, robot model, robot index 0 (main robot), gain (kp), weight (priority)
        self.qpsolver.addTask(self.comTask)

        # Reduce the posture task stiffness
        self.postureTask.stiffness(1)

        self.comDown = True
        self.comZero = eigen.Vector3d.Zero()

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
        return True
    
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

    def reset_callback(self, data):
        # In the reset callback, reset the task to the current CoM
        self.comTask.reset()
        self.comZero = self.comTask.com()
        pass
    
    #load the controller
    @staticmethod
    def create(robot, dt):
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot,env], dt)