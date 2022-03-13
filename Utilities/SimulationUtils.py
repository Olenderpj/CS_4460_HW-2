from Constants.EnvironmentConstants import *
from ObjectClasses.SceneShape import SceneShape


def getAllSceneShapes(sim, estimatedSceneShapes=1000):
    i = 0
    sceneObjects = []

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
        else:
            break
        i += 1
    return sceneObjects
