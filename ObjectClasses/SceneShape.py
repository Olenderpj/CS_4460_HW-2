import math
from Constants.EnvironmentConstants import *
from Constants.OutputConstants import PIXEL_SCALAR
from ObjectClasses.Pose import Pose
from ObjectClasses.PixelCoordinates import PixelCoordinates


class SceneShape:
    """
    Represents a Singular object in the Copellia Simulation Environment Ex: a wall, window, etc...
    Stores the objects properties like handle, bounding box, pose, and also has a sub-class
    containing the pixel values see the @PixelCoordinates for more
    """

    def __init__(self, simulation, objectHandle):
        self.floorHandle = simulation.getObject(FLOOR)
        self.orientation = round(math.degrees(simulation.getObjectOrientation(objectHandle, self.floorHandle)[2]))
        self.simulation = simulation
        self.floorHandle = simulation.getObject(FLOOR)
        self.objectHandle = objectHandle
        self.name = simulation.getObjectAlias(objectHandle, -1)
        self.shapeBoundingBox = simulation.getShapeBB(objectHandle)
        self.floorBoundingBox = simulation.getShapeBB(self.floorHandle)

        # Objects Height and width dimensions used for generatring the pygame square
        # Object is on the horizontal plane
        if self.orientation == 90 or self.orientation == -90:
            self.shapeBoundingBoxWidth = (self.shapeBoundingBox[1] * PIXEL_SCALAR) + OBJECT_BUFFER
            self.shapeBoundingBoxHeight = (self.shapeBoundingBox[0] * PIXEL_SCALAR) + OBJECT_BUFFER

        # Object is on the vertical plane
        elif self.orientation == 0 or self.orientation == -180 or self.orientation == 180:
            self.shapeBoundingBoxWidth = (self.shapeBoundingBox[0] * PIXEL_SCALAR) + OBJECT_BUFFER
            self.shapeBoundingBoxHeight = (self.shapeBoundingBox[1] * PIXEL_SCALAR) + OBJECT_BUFFER

        # Object position Relative to the floor
        #self.objectPosition = simulation.getObjectPosition(self.objectHandle, self.floorHandle)

        # Create a new pose object
        self.pose = Pose(self.simulation, self.objectHandle, self.floorHandle, self.shapeBoundingBox)
        self.pixelCoordinates = PixelCoordinates(self.pose.x1,
                                                 self.pose.y1,
                                                 self.pose.x2,
                                                 self.pose.y2,
                                                 self.floorBoundingBox[0],
                                                 self.floorBoundingBox[1])

    def __str__(self):
        return "ID: " + str(self.objectHandle) + " Name: " + self.name
