import warnings

warnings.filterwarnings("ignore")

from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.visualization import plot_workspace, plot_occupancy_grid
from pcg_gazebo.generators.creators import box_factory
from pcg_gazebo.utils import generate_random_string

world_gen = WorldGenerator()

world_gen.add_asset(
    tag='dyn_box',
    description=dict(
        type='box',
        args=dict(
            size="5 * __import__('pcg_gazebo').random.rand(3)",
            name='cuboid',
            mass="max(0.1, __import__('pcg_gazebo').random.rand())",
            color='xkcd'
        )
    )
)

# Check if models where included correctly
print('Asset is available for world generation=', 'dyn_box' in world_gen.assets.tags)

world_gen.add_asset(
    tag='static_cylinder',
    description=dict(
        type='cylinder',
        args=dict(
            length="2 * __import__('pcg_gazebo').random.rand()",
            radius="2 * __import__('pcg_gazebo').random.rand()",
            name='cylinder',
            color='xkcd'
        )
    )
)

# Check if models where included correctly
print('Asset is available for world generation=', 'static_cylinder' in world_gen.assets.tags)

world_gen.add_constraint(
    name='tangent_to_ground_plane',
    type='tangent',
    frame='world',
    reference=dict(
        type='plane',
        args=dict(
            origin=[0, 0, 0],
            normal=[0, 0, 1]
        )
    )
)

print('List of constraints=', list(world_gen.constraints.tags))

world_gen.add_constraint(
    name='my_workspace',
    type='workspace',
    frame='world',
    geometry_type='area',
    points=[
        [-5, -5, 0],
        [-5, 5, 0],
        [5, 5, 0],
        [5, -5, 0],
    ]
)

print('List of constraints=', list(world_gen.constraints.tags))

plot_workspace(workspace=world_gen.constraints.get('my_workspace'))

floor_model = box_factory(
    size=[
        [20, 20, 0.01]
    ],
    mass=1,
    use_permutation=True,
    name='box_floor'
)[0]
floor_model.name = 'box_floor'

world_gen.add_asset(tag='box_floor', description=floor_model)

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='gp_engine',
    models=['box_floor'],
    poses=[[0, 0, -0.02, 0, 0, 0]])
world_gen.set_model_as_ground_plane('box_floor')
# world_gen.run_engines()

NUM_BOXES = 3
NUM_CYLINDER = 3

placement_policy = dict(
    models=['dyn_box', 'static_cylinder'],
    config=[
        dict(
            dofs=['x', 'y'],            
            tag='workspace',
            workspace='my_workspace'
        ),
        dict(
            dofs=['yaw'],            
            tag='uniform',                            
            min=-3.141592653589793,
            max=3.141592653589793               
        )
    ]
)

world_gen.add_engine(
    tag='box_placement',
    engine_name='random_pose',
    models=['dyn_box', 'static_cylinder'],
    max_num=dict(
        dyn_box=NUM_BOXES,
        static_cylinder=NUM_CYLINDER),
    model_picker='random',
    no_collision=True,
    policies=[placement_policy],
    constraints=[
        dict(
            model='dyn_box',
            constraint='tangent_to_ground_plane'),
        dict(
            model='static_cylinder',
            constraint='tangent_to_ground_plane')
    ]
)


# world_gen.init()
world_gen.run_engines()
print(world_gen.world.models.keys())
world_gen.world.create_scene().show()

import matplotlib.pyplot as pyplot
fig = plot_occupancy_grid(world_gen.world.models, with_ground_plane=False, static_models_only=False, ground_plane_models=['box_floor'])
pyplot.show()

fig = plot_occupancy_grid(
    world_gen.world.models,
    with_ground_plane=False,
    static_models_only=True,
    ground_plane_models=['box_floor'])
pyplot.show()

fig = world_gen.world.plot_footprints(engine='matplotlib')
pyplot.show()