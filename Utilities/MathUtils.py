import math


def mapEntityOrientationFromRadiansToDegrees(orientationObj):
    newOrientationList = []
    for i in range(len(orientationObj)):
        # Convert radians to degrees
        if orientationObj[i] < 0:
            result = math.degrees(orientationObj[i]) % -360
        else:
            result = math.degrees(orientationObj[i]) % 360

        # Adjust the result by 10000 if the string of the value contains an E
        # This might have to be adjusted later
        if "e" in str(orientationObj[i]):
            result = result * 100000
        newOrientationList.append(result)
    return newOrientationList


def determineQuadrant(x, y):
    if x > 0 and y > 0:
        return 1
    elif x < 0 and y > 0:
        return 2
    elif x < 0 and y < 0:
        return 3
    elif x > 0 and y < 0:
        return 4


# Normalizes a value within the specified range
def normalizePointValue(value, maxValue):
    half = maxValue / 2
    negHalf = half * -1
    normalizedValue = half + value

    if normalizedValue > maxValue:
        normalizedValue = maxValue
    elif normalizedValue < 0:
        normalizedValue = 0

    return normalizedValue