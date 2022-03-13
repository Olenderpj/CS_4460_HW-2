from Constants.OutputConstants import PIXEL_SCALAR
from Constants.EnvironmentConstants import OBJECT_BUFFER
from Utilities.MathUtils import normalizePointValue


class PixelCoordinates:
    """
    Calculates the exact location for where an object should be placed within a PIL
    image that is generated later on.

    - lowerleft* is considered the coordinate of the actual point on the object in the scene

    - draw* is considered the calculation for where the object should be drawn in a PIL image.
    """

    def __init__(self, x1, y1, x2, y2, maxX, maxY):

        self.lowerLeftX1 = normalizePointValue(x1, maxX) * PIXEL_SCALAR
        #self.lowerLeftY1 = normalizePointValue(y1, maxY) * PIXEL_SCALAR
        #self.lowerLeftX2 = normalizePointValue(x2, maxX) * PIXEL_SCALAR
        self.lowerLeftY2 = normalizePointValue(y2, maxY) * PIXEL_SCALAR
        self.drawX1 = self.lowerLeftX1 + OBJECT_BUFFER
        #self.drawY1 = (maxY * PIXEL_SCALAR) - self.lowerLeftY1
        #self.drawX2 = self.lowerLeftX2
        self.drawY2 = ((maxY * PIXEL_SCALAR) - self.lowerLeftY2) + OBJECT_BUFFER

    def __str__(self):
        return "X1:" + str(self.drawX1) + \
               "\nY2:" + str(self.drawY2)
               #"\nY1:" + str(self.drawY1) +\
                #"\nX2:" + str(self.drawX2) + \

