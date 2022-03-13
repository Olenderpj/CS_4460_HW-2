from Constants.OutputConstants import PIXEL_SCALAR
import logging
import math


class Pose:
    def __init__(self, simulation, objectHandle, floorHandle, boundingBox):
        self.orientation = round(math.degrees(simulation.getObjectOrientation(objectHandle, floorHandle)[2]))
        self.pose = simulation.getObjectPose(objectHandle, floorHandle)
        self.x = self.pose[0]
        self.y = self.pose[1]
        self.z = self.pose[2]
        self.qx = self.pose[3]
        self.qy = self.pose[4]
        self.qz = self.pose[5]
        self.qw = self.pose[6]
        # self.name = simulation.getObjectAlias(objectHandle, -1)

        # Object is on the horizontal plane
        if self.orientation == 90 or self.orientation == -90:
            self.x1 = self.x - (boundingBox[1] / 2)
            self.y1 = self.y - (boundingBox[0] / 2)
            self.x2 = self.x + (boundingBox[1] / 2)
            self.y2 = self.y + (boundingBox[0] / 2)
        # Object is on the vertical plane
        elif self.orientation == 0 or self.orientation == -180 or self.orientation == 180:
            self.x1 = self.x - (boundingBox[0] / 2)
            self.y1 = self.y - (boundingBox[1] / 2)
            self.x2 = self.x + (boundingBox[0] / 2)
            self.y2 = self.y + (boundingBox[1] / 2)

        else:

            logging.critical(
                f"[CRITICAL ERROR]: An object has been found with an invalid orientation [NAME:{self.name}, ORIENTATION:{self.orientation}]")

    def getShape(self):

        return [abs(self.x1) * PIXEL_SCALAR,
                abs(self.y1) * PIXEL_SCALAR,
                abs(self.x2) * PIXEL_SCALAR,
                abs(self.y2) * PIXEL_SCALAR]

    def printShape(self):
        print([abs(self.x1),
               abs(self.y1),
               abs(self.x2),
               abs(self.y2)])

    def getValues(self):
        return str(self.x1) + " " + \
               str(self.y1) + " " + \
               str(self.x2) + " " + \
               str(self.y2)
