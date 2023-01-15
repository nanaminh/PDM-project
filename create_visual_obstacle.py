
import pybullet as p
import time
import math
import pybullet_data



cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
# p.loadURDF("cube_small.urdf",
#                        [0, 1, .1],
#                        p.getQuaternionFromEuler([0, 0, 0]),
#                        physicsClientId=self.CLIENT
#                        )
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.loadURDF("windowsVertical.urdf",
                   [0, 2, .5],
                   p.getQuaternionFromEuler([0,0,0])
                   )

shift = [0, 0, 0]
meshScale = [1, 1, 1]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="cube.obj",
                                    rgbaColor=[1, 1, 1, 0.5],
                                    specularColor=[0.4, .4, 0],
                                    meshScale=meshScale)
# collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
#                                           fileName="duck_vhacd.obj",
#                                           collisionFramePosition=shift,
#                                           meshScale=meshScale)

rangex = 1
rangey = 1
# for i in range(rangex):
#   for j in range(rangey):
#     p.createMultiBody(baseMass=0,
#                       baseInertialFramePosition=[0, 0, 0],
#                       baseVisualShapeIndex=visualShapeId,
#                       basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
#                                     (-rangey / 2 + j) * meshScale[1] * 2, 1],
#                       useMaximalCoordinates=True)
    
p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0,0,0],
                      useMaximalCoordinates=True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

p.addUserDebugLine([0,0,0], [1,0,0], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
currentColor = 0

while (1):
  time.sleep(1./240.)