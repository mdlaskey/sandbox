from sys_globals import *
from gripper import RobotGripper

class RobotArm:
    def __init__(self, transform, joint_angles):
        self.transform_ = transform

        self.defaultAngles_ = [0.0, 0.0, 0.0]
        if joint_angles is None: joint_angles = self.defaultAngles_
        self.jointAngles_ = joint_angles
        self.transform_ = b2Transform()
        self.transform_.angle = 0.0
        self.transform_.position = (0, 0)
        
        self.w = 2.0
        self.h = 7.5
        w = self.w
        h = self.h

        # Link One
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        transform.position = (0, h/2.0)
        self.L1_ = Polygon(vertices, transform)

        # Link Two
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 7)
        transform.position = (0, 3*h/2.0)
        self.L2_ = Polygon(vertices, transform)

        # Link 3
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 14)
        transform.position = (0, 5*h/2.0)
        self.L3_ = Polygon(vertices, transform)

        # Link 4
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 21)
        transform.position = (0, 7*h/2.0)
        self.L4_ = Polygon(vertices, transform)  

        
        self.joint1_ = RobotRevoluteJoint((0, h), self.jointAngles_[0], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joint2_ = RobotRevoluteJoint((0, 2*h), self.jointAngles_[1], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joint3_ = RobotRevoluteJoint((0, 3*h), self.jointAngles_[2], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joints_ = [self.joint1_, self.joint2_, self.joint3_]

        self.gripper_ = RobotGripper(self.transform_, h, w)


    def addToWorld(self, world):
        L1 = self.L1_.addToWorld(world, dynamic=False)
        L2 = self.L2_.addToWorld(world)
        L3 = self.L3_.addToWorld(world)
        L4 = self.L4_.addToWorld(world)
        joint1 = self.joint1_.addToWorld(world, self.L1_, self.L2_)
        joint2 = self.joint2_.addToWorld(world, self.L2_, self.L3_)
        joint3 = self.joint3_.addToWorld(world, self.L3_, self.L4_)

        gripper = self.gripper_.addToWorld(world)

        self.wj_ = world.CreateWeldJoint(
            bodyA=L4,
            bodyB=self.gripper_.getPalm(),
            anchor=self.gripper_.getPalm().position
            )

        self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=0.1,
                angularDamping=0.1
                )
        return self.body_

    def update(self):
        self.joint1_.update()
        self.joint2_.update()
        self.joint3_.update()
        self.gripper_.update()

    def getInvKin(self, xy): 
        
        if xy[0] < 0.0:
            self.jointAngles_ = [0.0, math.pi/3.0, math.pi/3.0]
        else:
            self.jointAngles_ = [0.0, -math.pi/3.0, -math.pi/3.0]
        

        def distanceToDefault(angles, *args):
            sum = np.sum([math.fabs(math.acos(np.dot(np.array([-math.sin(a_i), math.cos(a_i)]),
                                             np.array([-math.sin(d_i), math.cos(d_i)])))*.00001)
                                      for a_i, d_i in zip(angles, self.jointAngles_)])

            return sum

        def findEndEffector(angles, xy):
            start = np.matrix([[0.0],[0.0],[1.0]])
            theta = self.L1_.getAngle()
            position = self.L1_.getPosition()
            T_w = np.matrix([[math.cos(theta),-math.sin(theta),position[0]],
                            [math.sin(theta),math.cos(theta),self.h],
                            [0,0,1.0]])

            #print(str(angles*180/math.pi))

            endEffector = T_w * self.joint1_.getParameterMatrix(angles[0])*\
                self.joint2_.getParameterMatrix(angles[1])*\
                (self.joint3_.getParameterMatrix(angles[2])+\
                     self.gripper_.getParameterMatrix(angles[2]))* start

            #print(str(endEffector))
            s = np.abs(np.array([endEffector[0,0] - xy[0], endEffector[1,0] - xy[1]]))
            
            #print(str(s))

            return s

        return scipy.optimize.fmin_slsqp(func=distanceToDefault, x0=self.jointAngles_,
                                         f_eqcons=findEndEffector, args=(xy,), iprint=0,
                                         bounds=[(-math.pi,math.pi),(-math.pi,math.pi),
                                                 (-math.pi,math.pi)])


    def changeOrientation(self, angle):
        self.jointAngles_[-1] += angle

    def setTargetAngles(self, target_angles):
        self.jointAngles_ = target_angles
        for i in range(len(target_angles)):
            self.joints_[i].setTargetAngle(target_angles[i])

    def getTargetAngles(self):
        return np.array(self.jointAngles_)

    def getGripper(self):
        return self.gripper_

    def getCurrentAngles(self):
        currAngles = [joint.getAngle() for joint in self.joints_]
        return currAngles

    def ready(self):
        return set(round(self.jointAngles_, 2)) == set(round(self.getCurrentAngles, 2))
