from pydrake.geometry import StartMeshcat, SceneGraph, MeshcatVisualizer

from pydrake.systems.framework import DiagramBuilder

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, AddMultibodyPlant
from pydrake.multibody.parsing import Parser



# from pydrake.manipulation.kuka_iiwa import *

meshcat = StartMeshcat()


builder = DiagramBuilder()

# begin of plant and scenegraph init
# plant = AddMultibodyPlant(builder)
# scenegraph = SceneGraph()
# AddDefaultVisualization(builder)
meshcat.Delete()
meshcat.DeleteAddedControls()

# Adds both MultibodyPlant and the SceneGraph, and wires them together.
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
# Note that we parse into both the plant and the scene_graph here.
Parser(plant, scene_graph).AddModelFromFile(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"))
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))
plant.Finalize()


visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

diagram = builder.Build()

context = diagram.CreateDefaultContext()
diagram.Publish(context)

while (1):
    pass

import pydot
(graph,) = pydot.graph_from_dot_data(diagram.GetGraphvizString())
graph.write_png('out.png') 
