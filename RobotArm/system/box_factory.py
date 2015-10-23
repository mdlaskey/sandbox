from sys_globals import *

class BoxFactory:
    def __init__(self):
        self.boxes = []

    def createNewBox(self, h, w, transform, startPos, startAngle, world):
        transform = transform
        transform = b2Transform()
        transform.angle = startAngle
        transform.position = startPos

        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        box = Polygon(vertices, transform)
        self.boxes.append(box)
        
        box.addToWorld(world)
        body = world.CreateDynamicBody(
            angle=transform.angle,
            position=transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return body

    def createTarget(self, r, transform, startPos, startAngle, world):
        transform = transform
        transform = b2Transform()
        transform.angle = startAngle
        transform.position = startPos

        self.target = Polygon(vertices=None,transform=transform,radius=r)
        self.target.addToWorld(world)
        body = world.CreateDynamicBody(
            angle=transform.angle,
            position=transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return body 

    def getTarget(self):
        return self.target

    def getBoxes(self):
        return self.boxes

    def getStateBox(self, box):
        return np.hstack((self.boxes[box].getPosition(), self.boxes[box].getAngle()))

    '''def getStateBoxes(self):
        target = np.array([self.target.getPosition(), self.target.getAngle()])
        objects = np.zeros(len(self.boxes)+1, dtype=object)
        objects[0] = target
        for i in range(len(self.boxes)):
            objects[i+1] = self.getStateBox(i)
        return objects'''

    '''def getBoxesOffTable(self, table):
        outOfBounds = []
        transform = b2Transform()
        eIn1 = b2RayCastInput(p1=table[0], p2=table[1], maxFraction=1)
        eOut1 = b2RayCastOutput()
        
        eIn2 = b2RayCastInput(p1=table[1], p2=table[2], maxFraction=1)
        eOut2 = b2RayCastOutput()

        eIn3 = b2RayCastInput(p1=table[2], p2=table[3], maxFraction=1)
        eOut3 = b2RayCastOutput()
        
        eIn4 = b2RayCastInput(p1=table[3], p2=table[0], maxFraction=1)
        eOut4 = b2RayCastOutput()
        for box in self.boxes:
            hit1 = box.fixtures.RayCast(eOut1, eIn1, transform, 0)
            if hit1 or hit2 or hit3 or hit4:
                outOfBounds.append(box)
            else:'''                
            
    
    def getStateBoxes(self):
        objects = np.hstack((self.target.getPosition(), self.target.getAngle()))
        for i in range(len(self.boxes)):
            objects = np.hstack((objects, self.getStateBox(i)))
        return objects

    def resetBoxes(self, world):
        for box in self.boxes:
            world.DestroyBody(box.getBody())
        world.DestroyBody(self.target.getBody())
        self.boxes = []
