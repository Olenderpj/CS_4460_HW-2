from Constants.EnvironmentConstants import *
from ObjectClasses.SceneShape import SceneShape


def getAllSceneShapes(sim, estimatedSceneShapes=1000):
    i = 0
    sceneObjects = []
    outputFile = open("EnvironmentObstacles/Obstacles.txt", "w")

    while i < estimatedSceneShapes:
        objectHandle = sim.getObjects(i, sim.object_shape_type)

        if objectHandle != -1:
            print(f"[Retrieval]: Retrieving object {objectHandle} from the Scene")

            shape = SceneShape(sim, objectHandle)

            if shape.name not in EXCLUDED_SCENE_OBJECTS:
                sceneObjects.append((shape.pixelCoordinates.drawX1,
                                     shape.pixelCoordinates.drawY2,
                                     shape.shapeBoundingBoxWidth,
                                     shape.shapeBoundingBoxHeight))

                outputFile.write(str(round(shape.pixelCoordinates.drawX1, 7)) + " " +
                                 str(round(shape.pixelCoordinates.drawY2, 7)) + " " +
                                 str(round(shape.shapeBoundingBoxWidth, 7)) + " " +
                                 str(round(shape.shapeBoundingBoxHeight, 7)) + "\n")
        else:
            break
        i += 1
    return sceneObjects


def readSceneObjectsFromFile():
    objFile = open("EnvironmentObstacles/Obstacles.txt", "r")
    objList = []

    for line in objFile:
        line = line.replace("\n", "")
        dimensions = line.split(" ")
        objList.append((float(dimensions[0]),
                        float(dimensions[1]),
                        float(dimensions[2]),
                        float(dimensions[3])))
    return objList
