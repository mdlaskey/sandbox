class RobotRevoluteJoint:
    def __init__(self, anchor=(0,0), target_angle=0.0,
                    lower_angle=-math.pi/2.0, upper_angle=math.pi/2.0, max_motor_torque=0.0,
                    limit_enabled=True, motor_enabled=True):
        self.anchor_ = anchor
        self.targetAngle_ = target_angle
        self.lowerAngle_ = lower_angle
        self.upperAngle_ = upper_angle
        self.maxMotorTorque_ = max_motor_torque
        self.limitEnabled_ = limit_enabled
        self.motorEnabled_ = motor_enabled

        self.controller_ = PController()

    def addToWorld(self, world, bodyA, bodyB):
        self.bodyA_ = bodyA
        self.bodyB_ = bodyB
        self.rj_ = world.CreateRevoluteJoint(
            bodyA=bodyA.getBody(),
            bodyB=bodyB.getBody(),
            anchor=self.anchor_,
            lowerAngle=self.lowerAngle_,
            upperAngle=self.upperAngle_,
            motorSpeed=0.0,
            maxMotorTorque=self.maxMotorTorque_,
            enableLimit=self.limitEnabled_,
            enableMotor=self.motorEnabled_
            )
        
        return self.rj_

    def getTargetAngleDegrees(self):
        return self.targetAngle_*180/math.pi
    
    def setTargetAngle(self, new_angle):
        self.targetAngle_= new_angle
        return self.targetAngle_

    def getReferenceAngle(self):
        referenceAngle = self.bodyB_.getAngle() - self.bodyA_.getAngle()
        return referenceAngle

    def getMotorSpeed(self):
        return self.rj_.motorSpeed

    def setMotorSpeed(self, new_speed):
        self.rj_.motorSpeed = new_speed
        return self.rj_.motorSpeed

    def getCurrentAngleDegrees(self):
        return self.rj_.angle*180/math.pi

    def getAngle(self):
        return self.rj_.angle

    def update(self):
        if round(self.getCurrentAngleDegrees(), 2) != round(self.getTargetAngleDegrees(), 2):
            p_value = self.controller_.control(self.rj_.angle, self.targetAngle_)
            motorSpeed = self.setMotorSpeed(p_value*20)
        else:
            self.setMotorSpeed(0.0)

    def getParameterMatrix(self, angle):
        t_x = -math.sin(angle) * self.bodyB_.getHeight()
        t_y = math.cos(angle) * self.bodyB_.getHeight()
        parameters = np.matrix([[math.cos(angle), -math.sin(angle), t_x],
                                [math.sin(angle), math.cos(angle), t_y],
                                [0, 0, 1]])
        return parameters
