
FLOOR = "/Floor/element"
#FLOOR = "/ResizableFloor_5_25"
EXCLUDED_SCENE_OBJECTS = ["Floor", "Floor/element", "box", "element", "visibleElement", "DefaultLights", "XYZCameraProxy", "DefaultCamera"]
NODE_COLOR = "Red"

# Node radius should be smaller than the spacing
NODE_SPACING = .73 #.03
NODE_RADIUS = .02 #.02
OBJECT_BUFFER = 10
# Can be 0 or 1.
#   0 creates new scene objects to the file
#   1 uses existing objects in the file
OBJECT_TEXT_FILE_FLAG = 1
RRT_NODE_SPACING = 7.5

#Starts from the upper right hand corner

# Course flag determines whether the maze or the obstacle course will be executed
# 0 - maze
# 1 - Obstacle Course
COURSE_FLAG = 0
# Start flag determines what set of x*, y* pairs to use.
MAZE_START_FLAG = 3
OC_START_FLAG = 3

# -------------------------

MAZE_START_X1 = 665
MAZE_START_Y1 = 475

MAZE_START_X2 = 504
MAZE_START_Y2 = 685

MAZE_START_X3 = 685
MAZE_START_Y3 = 752

MAZE_END_X = 460
MAZE_END_Y = 210

# -------------------------
# Obstacle course coordinates

OC_START_X1 = 485
OC_START_Y1 = 210

OC_START_X2 = 135
OC_START_Y2 = 130

OC_START_X3 = 200
OC_START_Y3 = 370

OC_END_X = 300
OC_END_Y = 485