from sys_globals import *

class RobotGripper:
    def __init__(self, transform, height, width):
        self.transform_ = transform
        self.transform_ = b2Transform()
        self.transform_.angle = transform.angle

        self.h_ = height
        self.w_ = width
        h = self.h_
        w = self.w_
        w_small = w/3.0
        h_small = h/2.5

        self.lengthPalm = h/1.3

        self.transform_.position = (0, 8*h/2.0 + w_small/2.0)
        
        # Gripper Palm
        vertices = vertices=[(0,0),(self.lengthPalm,0),(self.lengthPalm,w_small),(0,w_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 28.4167)
        transform.position = (0, 8*h/2.0 + w_small/2.0)
        self.gripperPalm_ = Polygon(vertices, transform)

        # Gripper Left
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (-3, 27.25)
        x = w/2.0 + ((self.lengthPalm-w)/2.0 - w_small) + w_small/2.0
        y = 8*h/2.0 + w_small + h_small/2.0
        transform.position = (-x, y)
        print(transform.position)
        self.gripperLeft_ = Polygon(vertices, transform)

        # Gripper Right
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        # position = (2.75, 27.25)
        transform.position = (x, y)
        self.gripperRight_ = Polygon(vertices, transform)


        self.jointLeft_ = RobotPrismaticJoint(axis=(1,0), lower_translation=0.0, 
                                              upper_translation=self.lengthPalm-3*w_small/2.0,
                                              motor_force=90.0, enable_motor=True)
        self.jointRight_ = RobotPrismaticJoint(axis=(1,0), 
                                               lower_translation=-(self.lengthPalm-3*w_small/2.0),
                                               upper_translation=0.0, motor_force=90.0,
                                               enable_motor=True)

    def addToWorld(self, world):
        self.palm_ = self.gripperPalm_.addToWorld(world)
        self.left_ = self.gripperLeft_.addToWorld(world)
        self.right_ = self.gripperRight_.addToWorld(world)
        jointLeft = self.jointLeft_.addToWorld(world, self.palm_, self.left_)
        jointRight = self.jointRight_.addToWorld(world, self.palm_, self.right_)

        self.body_ = world.CreateDynamicBody(
            angle=self.transform_.angle,
            position=self.transform_.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return self.body_

    def getPalm(self):
        return self.palm_

    def getGrippers(self):
        return [self.left_, self.right_]

    def getWorldCenter(self):
        return self.body_.worldCenter

    def update(self):
        self.jointLeft_.update()
        self.jointRight_.update()

    def open(self):
        self.jointLeft_.open()
        self.jointRight_.open()
    
    def close(self):
        self.jointLeft_.close()
        self.jointRight_.close()

    def reset(self):
        self.jointLeft_.resetState()
        self.jointRight_.resetState()

    def getCenterPosition(self):
        return (0, self.left_.position[1])

    def getState(self):
        return self.jointLeft_.getState()

    def getParameterMatrix(self, angle):
        t_x=-math.sin(angle)*(self.gripperPalm_.getHeight() + self.gripperLeft_.getHeight()/2.0)
        t_y=math.cos(angle)*(self.gripperPalm_.getHeight() + self.gripperLeft_.getHeight()/2.0)
        parameters = np.matrix([[0, 0, t_x],
                                [0, 0, t_y],
                                [0, 0, 0]]) 

        return parameters
