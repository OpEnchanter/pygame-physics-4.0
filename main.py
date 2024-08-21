# Import modules
import pygame, sys, random, time, math

# Utility Functions
def getNearestPointOnLine(slope = float, yIntercept = float, pointX = float, pointY = float):
    if slope == 0:
        slope = 0.000001
    slp = slope
    yin = yIntercept
    ptx = pointX
    pty = pointY
    inx = (-1/slp+(pty+1/slp*ptx)-yin)/slp
    iny = -1/slp*(inx-ptx)+pty

    return (inx, iny)

def dist(pt1 = tuple, pt2 = tuple):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((500, 500))

physicsObjects = {}

class constraint():
    class rigid(object):
        def __init__(self, idx0, idx1):
            self.constraintTypeIndex = 0
            self.pt0Index = idx0
            self.pt1Index = idx1
    class spring(object):
        def __init__(self, elasticity, idx0, idx1):
            self.constraintTypeIndex = 1
            self.elasticity = elasticity
            self.pt0Index = idx0
            self.pt1Index = idx1

class collision(object):
    def __init__(self, otherObjID = str, location = tuple):
        self.collisionObjId = otherObjID
        self.collisionLocation = location


def generateObjectID():
    objId = str(random.randint(1111111111, 9999999999))
    while objId in physicsObjects.keys():
        objId = str(random.randint(1111111111, 9999999999))
    return objId

def createPhysicsObject(collisionMesh = list[tuple], constraints = list[constraint], worldPos = tuple, velocity = list[tuple], density = float):
    meshPointWorldLocations = []
    for localPosition in collisionMesh:
        meshPointWorldLocations.append((worldPos[0] + localPosition[0], worldPos[1] + localPosition[1]))

    physicsObjects[generateObjectID()] = {
        "collisionMesh": collisionMesh,
        "meshConstraints": constraints,
        "worldPosition": worldPos,
        "meshPointWorldLocations": meshPointWorldLocations,
        "density": density,
        "velocity": velocity
    }

def getMeshCollisions(objId = str, otherObjId = str):
    colliding = False
    quadTopLeft = []
    quadTopRight = []
    quadBottomLeft = []
    quadBottomRight = []
    for pt in physicsObjects[objId]["meshPointWorldLocations"]:
        for edge in physicsObjects[otherObjId]["meshConstraints"]:
            # TopLeft
            if (physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0] < 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1] > 0 or 
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] < 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] > 0):

                slopeComp2 = (physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0])
                if slopeComp2 == 0:
                    slopeComp2 = 0.00000001

                slope = ((physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1]) / slopeComp2)

                nearestPtCoords = getNearestPointOnLine(slope, 0, pt[0], pt[1])

                otherObjCenter = physicsObjects[otherObjId]["worldPosition"]

                if dist(otherObjCenter, pt) < dist(otherObjCenter, nearestPtCoords):
                    quadTopLeft.append(True)
                else:
                    quadTopLeft.append(False)
            
            # TopRight
            if (physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0] > 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1] > 0 or 
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] > 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] > 0):

                slopeComp2 = (physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0])
                if slopeComp2 == 0:
                    slopeComp2 = 0.00000001

                slope = ((physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1]) / slopeComp2)
                    

                nearestPtCoords = getNearestPointOnLine(slope, 0, pt[0], pt[1])

                otherObjCenter = physicsObjects[otherObjId]["worldPosition"]

                if dist(otherObjCenter, pt) < dist(otherObjCenter, nearestPtCoords):
                    quadTopRight.append(True)
                else:
                    quadTopRight.append(False)
            
            # BottomLeft
            if (physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0] < 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1] < 0 or 
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] < 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] < 0):

                slopeComp2 = (physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0])
                if slopeComp2 == 0:
                    slopeComp2 = 0.00000001

                slope = ((physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1]) / slopeComp2)

                nearestPtCoords = getNearestPointOnLine(slope, 0, pt[0], pt[1])

                otherObjCenter = physicsObjects[otherObjId]["worldPosition"]

                if dist(otherObjCenter, pt) < dist(otherObjCenter, nearestPtCoords):
                    quadBottomLeft.append(True)
                else:
                    quadBottomLeft.append(False)
                
            # BottomRight
            if (physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0] > 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1] < 0 or 
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] > 0 and
                physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] < 0):

                slopeComp2 = (physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0])
                if slopeComp2 == 0:
                    slopeComp2 = 0.00000001

                slope = ((physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1]) / slopeComp2)

                nearestPtCoords = getNearestPointOnLine(slope, 0, pt[0], pt[1])

                otherObjCenter = physicsObjects[otherObjId]["worldPosition"]

                if dist(otherObjCenter, pt) < dist(otherObjCenter, nearestPtCoords):
                    quadBottomRight.append(True)
                else:
                    quadBottomRight.append(False)
                
    if not False in quadTopLeft and not False in quadTopRight and not False in quadBottomLeft and not False in quadBottomRight:
        colliding = True


    return colliding


def renderObjects():
    for objId in physicsObjects.keys():
        objPts = physicsObjects[objId]["meshPointWorldLocations"]

        pygame.draw.polygon(win, (255, 255, 255), objPts)





createPhysicsObject(
    [(-35, 35), (35, 35), (35, -35), (-35, -35)], 
    [constraint.rigid(0, 1), constraint.rigid(1, 2), constraint.rigid(2, 3), constraint.rigid(3, 0)], 
    (215, 215), 
    [(0,0), (0,0), (0,0), (0,0)], 
    1
)

createPhysicsObject(
    [(-35, 35), (35, 35), (35, -35), (-35, -35)], 
    [constraint.rigid(0, 1), constraint.rigid(1, 2), constraint.rigid(2, 3), constraint.rigid(3, 0)], 
    (250, 250), 
    [(0,0), (0,0), (0,0), (0,0)], 
    1
)

#print(physicsObjects)

print(getMeshCollisions(list(physicsObjects.keys())[0], list(physicsObjects.keys())[1]))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    win.fill((55, 55, 55))

    renderObjects()
    print(getMeshCollisions(list(physicsObjects.keys())[0], list(physicsObjects.keys())[1]))

    pygame.display.flip()

pygame.quit()
sys.exit()