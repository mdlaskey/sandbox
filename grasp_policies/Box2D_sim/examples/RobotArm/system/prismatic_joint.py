class RobotPrismaticJoint():
    def __init__(self, axis=(1,0), lower_translation=0.0, upper_translation=0.0,
                 enable_limit=True, motor_force=0.0, enable_motor=True, roc=0.03,
                 gripper_state=0.0):
        self.axis_ = axis
        self.lowerTranslation_ = lower_translation
        self.upperTranslation_ = upper_translation
        self.enableLimit_ = enable_limit
        self.motorForce_ = motor_force
        self.enableMotor_ = enable_motor
        self.roc = roc
        self.controller_= PController()
        
        self.gripperState = gripper_state

    def addToWorld(self, world, bodyA, bodyB):
        self.pj_ = world.CreatePrismaticJoint(
            bodyA=bodyA,
            bodyB=bodyB,
            anchor=bodyB.worldCenter,
            axis=self.axis_,
            lowerTranslation=self.lowerTranslation_,
            upperTranslation=self.upperTranslation_,
            enableLimit=self.enableLimit_,
            maxMotorForce=self.motorForce_,
            motorSpeed=0.0,
            enableMotor=self.enableMotor_
            )

    def getTranslation(self):
        return self.pj_.translation

    def getSpeed(self):
        return self.pj_.speed

    def getMotorSpeed(self):
        return self.pj_.motorSpeed
    
    def getState(self):
        return self.gripperState

    def resetState(self):
        self.gripperState = 0.0

    def close(self):
        if (self.gripperState >= 1.0 - self.roc):
            self.gripperState = 1.0
        else:
            self.gripperState += self.roc

    def open(self):
        if (self.gripperState <= 0.0 + self.roc):
            self.gripperState = 0.0
        else:
            self.gripperState -= self.roc

    def update(self):
        xp = [0, 1]
        if (self.upperTranslation_ <= 0.0):
            fp = [self.upperTranslation_, self.lowerTranslation_/2.0]
        else:
            fp = [self.lowerTranslation_, self.upperTranslation_/2.0]
        i = np.interp(self.gripperState, xp, fp)

        if round(self.pj_.translation, 2) != round(i, 2):  
            p_value = self.controller_.control(self.pj_.translation, i)
            self.pj_.motorSpeed = p_value * 1.5
        else:
            self.pj_.motorSpeed=0.0

