import Box2D
from Box2D.b2 import *

class Polygon:
    def __init__(self, vertices=None, transform=b2Transform(),
                 massDensity=0.1, friction=0.5, linearDamping=10, angularDamping=10, center=True,
                 radius=None):
        self.transform_ = transform
        self.massDensity_ = massDensity
        self.friction_ = friction
        self.linearDamping_ = linearDamping
        self.angularDamping_ = angularDamping

        s = b2Separator()

        if vertices == None:
            self.radius = radius
            self.polyShape_ = b2CircleShape(pos=self.transform_.position,
                                            radius=self.radius)
            
        else:
            self.vertices_ = vertices
            vert_array = np.array(self.vertices_)
            self.centroid_ = list(np.mean(vert_array, axis=0))
            if center:
                vert_array_cent = vert_array - self.centroid_
                self.vertices_ = vert_array_cent.tolist()

            self.polyShape_ = b2PolygonShape(vertices=self.vertices_)
        

    @property
    def shape(self):
        return self.polyShape_

    def centerVertices(self, world, dynamic):
        """ Center the polygon vertices at the center of mass for more intuitive pose control """
        localCenter = self.body_.localCenter
#        self.body_.position = self.body_.position - localCenter

        # center the vertex list
        newVertices = []
        for v in self.vertices_:
            newVertices.append((v[0] - localCenter[0], v[1] - localCenter[1]))
        self.vertices_ = newVertices
        self.polyShape_.vertices = self.vertices_
 
        # re-add to the world
        world.DestroyBody(self.body_)
        self.addToWorld(world, center=False, dynamic=dynamic)
       
    def addToWorld(self, world, center = False, dynamic=True):
        """ Add the polygon to the given world as a dynamic body """
        if dynamic:
            self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )
        else:
            self.body_ = world.CreateStaticBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )


        if center:
            self.centerVertices(world, dynamic)
        
        return self.body_

    def getHeight(self):
        height = self.vertices_[2][1] - self.vertices_[1][1]
        return height

    def getWidth(self):
        width = self.vertices_[1][0] - self.vertices_[0][0]
        return width

    def getBody(self):
        return self.body_
    
    def setAngle(self, angle):
        self.body_.angle = angle
        return angle
    
    def getAngle(self):
        return self.body_.angle

    def getAngleDegrees(self):
        return self.body_.angle*180/math.pi

    def changePosition(self, position):
        self.body_.position = np.add(self.body_.position, position)
        return self.body_.position

    def changeAngle(self, change):
        self.body_.angle += change
        return self.body_.angle   

    def getPosition(self):
        return np.array([self.body_.position[0], self.body_.position[1]])

