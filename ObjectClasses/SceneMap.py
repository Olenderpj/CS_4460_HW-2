import pygame
from sys import exit
from Utilities.SimulationUtils import *
from Constants.OutputConstants import *




def mapScene(sim):
    allObstacles = getAllSceneShapes(sim)
    floor = sim.getObject(FLOOR)
    floorSize = sim.getShapeBB(floor)
    print(floorSize)

    pygame.init()
    surface = pygame.display.set_mode((floorSize[0] * PIXEL_SCALAR, floorSize[1] * PIXEL_SCALAR))
    surface.fill((255, 255, 255))
    color = (0, 0, 0)

    for i in allObstacles:
        if i.name not in EXCLUDED_SCENE_OBJECTS:
            print(i)
            obstacle = pygame.Rect(i.pixelCoordinates.drawX1,
                                   i.pixelCoordinates.drawY2,
                                   i.shapeBoundingBoxWidth,
                                   i.shapeBoundingBoxHeight)
            pygame.draw.rect(surface, color, obstacle)
    pygame.display.flip()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
