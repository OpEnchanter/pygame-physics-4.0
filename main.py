# Import modules
import pygame, sys, random, time, math

# Utility Functions
def getNearestPointOnLine(slope = float, yIntercept = float, pointX = float, pointY = float, lineX = float, lineY = float, lineMaxLen = float):
    if slope == 0:
        slope = 1
    if lineMaxLen == 0:
        lineMaxLen = 1
    slp = slope
    yin = yIntercept
    ptx = pointX
    pty = pointY
    inx = ((-1/slp+(pty+1/slp*ptx)-yin)/slp)+lineX
    iny = (-1/slp*(inx-ptx)+pty)%lineMaxLen+lineY

    return (inx, iny)

def dist(pt1 = tuple, pt2 = tuple):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((500, 500))

physicsObjects = {}

class constraint():
    class rigid(object):
        def __init__(self, idx0, idx1, length):
            self.constraintTypeIndex = 0
            self.pt0Index = idx0
            self.pt1Index = idx1
            self.len = length
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

def getMeshIntersections(objId = str, otherObjId = str):
    collidingPts = []
    for pt in physicsObjects[objId]["meshPointWorldLocations"]:
        insideEdges = []
        for edge in physicsObjects[otherObjId]["meshConstraints"]:            
            slopeComp2 = (physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][0] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][0])
            if slopeComp2 == 0:
                slopeComp2 = 0.00000001

            slope = ((physicsObjects[otherObjId]["collisionMesh"][edge.pt1Index][1] - physicsObjects[otherObjId]["collisionMesh"][edge.pt0Index][1]) / slopeComp2)
            
            lineMaxLen = dist(
                (physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt0Index][0], physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt0Index][1]),
                (physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt1Index][0], physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt1Index][1])
            )

            nearestPtCoords = getNearestPointOnLine(slope, 0, pt[0], pt[1], physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt0Index][0], physicsObjects[otherObjId]["meshPointWorldLocations"][edge.pt0Index][1], lineMaxLen)

            otherObjCenter = physicsObjects[otherObjId]["worldPosition"]

            if dist(otherObjCenter, pt) <= dist(otherObjCenter, nearestPtCoords):
                insideEdges.append(True)
            else:
                insideEdges.append(False)
                
        if not False in insideEdges:
            collidingPts.append(True)
            pygame.draw.circle(win, (0,0,255), pt, 10)
        else:
            collidingPts.append(False)

    return collidingPts

def calculateObjectVertexVelocity(objectId, intersectionData):
    for ptVel in physicsObjects[objectId]["velocity"]:
        newVel = [ptVel[0], ptVel[1]]
        newVel[1] += 0.000981
        if intersectionData[physicsObjects[objectId]["velocity"].index(ptVel)] == True:
            newVel[0] *= -1
            newVel[1] *= -1
        if physicsObjects[objectId]["meshPointWorldLocations"][physicsObjects[objectId]["velocity"].index(ptVel)][1] >= 500:
            newVel[0] *= -1
            newVel[1] *= -1
        if physicsObjects[objectId]["meshPointWorldLocations"][physicsObjects[objectId]["velocity"].index(ptVel)][0] <= 0:
            newVel[0] *= -1
            newVel[1] *= -1
        if physicsObjects[objectId]["meshPointWorldLocations"][physicsObjects[objectId]["velocity"].index(ptVel)][0] >= 500:
            newVel[0] *= -1
            newVel[1] *= -1

        physicsObjects[objectId]["velocity"][physicsObjects[objectId]["velocity"].index(ptVel)] = (newVel[0], newVel[1])
    


def calculateObjectVertexPositions(objectId):
    for pt in physicsObjects[objectId]["meshPointWorldLocations"]:
        objConstraint = physicsObjects[objectId]["meshConstraints"][physicsObjects[objectId]["meshPointWorldLocations"].index(pt)]
        otherPt = physicsObjects[objectId]["meshPointWorldLocations"][objConstraint.pt1Index]
        ptDist = objConstraint.len
        ptIdx = physicsObjects[objectId]["meshPointWorldLocations"].index(pt)
        otherPtIdx = physicsObjects[objectId]["meshPointWorldLocations"].index(otherPt)

        # Move object according to velocity
        physicsObjects[objectId]["meshPointWorldLocations"][ptIdx] = (pt[0] + physicsObjects[objectId]["velocity"][physicsObjects[objectId]["meshPointWorldLocations"].index(pt)][0],
              pt[1] + physicsObjects[objectId]["velocity"][physicsObjects[objectId]["meshPointWorldLocations"].index(pt)][1])

        pt = physicsObjects[objectId]["meshPointWorldLocations"][ptIdx]
        otherPt = physicsObjects[objectId]["meshPointWorldLocations"][otherPtIdx]

        # Move Connected point to make object not stretch
        ptVec = (otherPt[0] - pt[0], otherPt[1] - pt[1])
        ptVecMagnitude = math.sqrt(ptVec[0]**2 + ptVec[1]**2)
        if ptVecMagnitude != 0:
            ptVec = (ptVec[0] / ptVecMagnitude, ptVec[1] / ptVecMagnitude)
            ptVec = (ptVec[0] * ptDist, ptVec[1] * ptDist)
            mvPos = (
                ptVec[0] + pt[0],
                ptVec[1] + pt[1]
            )
            
            mvVec = (mvPos[0] - otherPt[0], mvPos[1] - otherPt[1])
            
            physicsObjects[objectId]["meshPointWorldLocations"][otherPtIdx] = (
                mvVec[0] + otherPt[0],
                mvVec[1] + otherPt[1]
            )
        



def renderObjects():
    for objId in physicsObjects.keys():
        objPts = physicsObjects[objId]["meshPointWorldLocations"]

        pygame.draw.polygon(win, (255, 255, 255), objPts)




createPhysicsObject(
    [(-35, 35), (35, 35), (35, -35)], 
    [constraint.rigid(0, 1, 70), constraint.rigid(1, 2, 70), constraint.rigid(2, 0, 70)], 
    (250, 250), 
    [(0,0), (0,0), (0,0)], 
    1
)

createPhysicsObject(
    [(-35, 35), (35, 35), (1, -35)], 
    [constraint.rigid(0, 1, 50), constraint.rigid(1, 2, 50), constraint.rigid(2, 0, 50)], 
    (450, 250), 
    [(0,0), (0,0), (0,0)], 
    1
)

#print(physicsObjects)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    win.fill((55, 55, 55))

    renderObjects()

    for key in list(physicsObjects.keys()):
        objectIntersections = []
        objectIntersectionData = []
        otherKeys = [item for item in list(physicsObjects.keys()) if item != key]
        for otherKey in otherKeys:
            objectIntersectionData.append(getMeshIntersections(key, otherKey))
        objectIntersections = objectIntersectionData[0]
        for intersectData in objectIntersectionData:
            for dataPoint in intersectData:
                if dataPoint == True:
                    objectIntersections[intersectData.index(dataPoint)] = True

        calculateObjectVertexVelocity(key, objectIntersections)
        calculateObjectVertexPositions(key)
        

    physicsObjects[list(physicsObjects.keys())[0]]["worldPosition"] = pygame.mouse.get_pos()
    objWorldLoc = physicsObjects[list(physicsObjects.keys())[0]]["worldPosition"]
    for pt in physicsObjects[list(physicsObjects.keys())[0]]["collisionMesh"]:
        physicsObjects[list(physicsObjects.keys())[0]]["meshPointWorldLocations"][physicsObjects[list(physicsObjects.keys())[0]]["collisionMesh"].index(pt)] = (pt[0] + objWorldLoc[0], pt[1] + objWorldLoc[1])

    pygame.display.flip()

pygame.quit()
sys.exit()