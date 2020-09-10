import warnings
warnings.filterwarnings("ignore")

from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.visualization import plot_occupancy_grid, plot_workspace
from pcg_gazebo import random
world_gen = WorldGenerator()

from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.generators.creators import extrude

wall_thickness = 0.15
wall_height = 2

wall_polygon = random_rectangles(
    n_rect=5, delta_x_min=15, delta_x_max=20, delta_y_min=15, delta_y_max=20)

walls_model = extrude(
    polygon=wall_polygon,
    thickness=wall_thickness,
    height=wall_height,
    pose=[0, 0, wall_height / 2., 0, 0, 0],
    extrude_boundaries=True,
    color='xkcd')
walls_model.name = 'walls'

print(walls_model.to_sdf())


world_gen.add_asset(
    tag=walls_model.name,
    description=walls_model
)

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='add_ground_plane',
    models=['ground_plane'],
    poses=[
        [0, 0, 0, 0, 0, 0]
    ]
)

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='add_walls',
    models=[walls_model.name],
    poses=[
        [0, 0, wall_height / 2., 0, 0, 0]
    ]
)

world_gen.run_engines()

# fig = world_gen.world.plot_footprints(engine='matplotlib')

print(world_gen.world.models)

# fig = plot_occupancy_grid(
#     world_gen.world.models,
#     with_ground_plane=True,
#     static_models_only=False,
#     exclude_contains=['ground_plane'],
#     ground_plane_models=[walls_model.name])

free_space_polygon = world_gen.world.get_free_space_polygon(
    ground_plane_models=[walls_model.name],
    ignore_models=['ground_plane'])

# Add the workspace constraint to the generator
world_gen.add_constraint(
    name='room_workspace',
    type='workspace',
    frame='world',
    geometry_type='polygon',
    polygon=free_space_polygon
)

# plot_workspace(world_gen.constraints.get('room_workspace'))

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

# Add cuboid model factory as an asset
world_gen.add_asset(
    tag='box',
    description=dict(
        type='box',
        args=dict(
            size="__import__('pcg_gazebo').random.rand(3)",
            mass="__import__('pcg_gazebo').random.rand()",
            name='cuboid',
            color='xkcd'
        )
    )
)

# Add sphere model factory as an asset
world_gen.add_asset(
    tag='sphere',
    description=dict(
        type='sphere',
        args=dict(
            radius="__import__('pcg_gazebo').random.rand()",
            mass="__import__('pcg_gazebo').random.rand()",
            name='sphere',
            color='xkcd'
        )
    )
)

# Add cylinder model factory as an asset
world_gen.add_asset(
    tag='cylinder',
    description=dict(
        type='cylinder',
        args=dict(
            radius="__import__('pcg_gazebo').random.rand()",
            length="max(0.5, __import__('pcg_gazebo').random.rand())",
            mass="__import__('pcg_gazebo').random.rand()",
            name='cylinder',
            color='xkcd'
        )
    )
)

NUM_CUBOIDS = 2
NUM_CYLINDER = 2
NUM_SPHERES = 2

placement_policy = dict(
    models=['box', 'cylinder', 'sphere'],
    config=[
        dict(
            dofs=['x', 'y'],            
            tag='workspace',
            workspace='room_workspace'
        ),
        dict(
            dofs=['roll', 'pitch', 'yaw'],        
            tag='uniform',                            
            min=-3.141592653589793,
            max=3.141592653589793               
        )
    ]
)

world_gen.add_engine(
    tag='object_placement',
    engine_name='random_pose',
    models=['box', 'cylinder', 'sphere'],
    max_num=dict(
        box=NUM_CUBOIDS,
        cylinder=NUM_CYLINDER,
        sphere=NUM_SPHERES),
    model_picker='random',
    no_collision=True,
    policies=[placement_policy],
    constraints=[
        dict(
            model='box',
            constraint='tangent_to_ground_plane'),
        dict(
            model='cylinder',
            constraint='tangent_to_ground_plane'),
        dict(
            model='sphere',
            constraint='tangent_to_ground_plane')
    ]
)

def generate_world():
    world_gen.init()
    world_gen.run_engines()

    print('List of models=', world_gen.world.models.keys())

    # fig = plot_occupancy_grid(world_gen.world.models, with_ground_plane=True, static_models_only=False, exclude_contains=['ground_plane'], ground_plane_models=[walls_model.name])
    return world_gen.world.copy()

print("--#################--")
print(world_gen.engines.tags)
generate_world()

world_gen.world.create_scene().show()

# for _ in range(3):
#     generate_world()

# SEED = random.randint(0, 10000)
# print('Seed=', SEED)
# world_gen.seed = SEED
# world_ref = generate_world()

# worlds = list()
# for _ in range(3):
#     worlds.append(generate_world())

# for i in range(len(worlds)):
#     print('Is world #{} equal to reference? '.format(i), world_ref == worlds[i])