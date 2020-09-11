import os
from pcg_gazebo.generators import ModelGroupGenerator

generator = ModelGroupGenerator('full_crate')

generator.add_asset(
    tag='crate',
    description=dict(
        type='mesh',
        args=dict(
            visual_mesh='file://' + os.path.abspath('meshes/crate.stl'),
            visual_mesh_scale=[1, 1, 1],
            use_approximated_collision=False,
            name='crate',
            color='xkcd'
        )
    )
)

generator.add_asset(
    tag='crate_ball',
    description=dict(
        type='sphere',
        args=dict(
            radius="max(0.05, 0.1 * __import__('numpy').random.random())",
            name='sphere',
            mass="max(0.1, __import__('numpy').random.random())",
            color='xkcd'
        )
    )
)

generator.add_asset(
    tag='crate_cuboid',
    description=dict(
        type='box',
        args=dict(
            size="0.1 * __import__('numpy').random.random(3)",
            name='cuboid',
            mass="max(0.01, __import__('numpy').random.random())",
            color='xkcd'
        )
    )
)

generator.add_asset(
    tag='crate_cylinder',
    description=dict(
        type='cylinder',
        args=dict(
            length="max(0.05, 0.1 *__import__('numpy').random.random())",
            radius="max(0.05, 0.1 *__import__('numpy').random.random())",
            name='cuboid',
            mass="max(0.01, __import__('numpy').random.random())",
            color='xkcd'
        )
    )
)

crate_assets = ['crate_ball', 'crate_cuboid', 'crate_cylinder']

generator.add_constraint(
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
print(dir(generator))
print(generator.constraints.tags)
generator.add_constraint(
    name='crate_base',
    type='workspace',
    geometry_type='area',
    frame='world',
    points=[ 
              [-0.5, -0.4, 0],
              [-0.5, 0.4, 0],
              [0.5, 0.4, 0],
              [0.5, -0.4, 0]
          ],
    geometry=dict( 
        geometry_type='area',
        description=dict(
          points=[ 
              [-0.5, -0.4, 0],
              [-0.5, 0.4, 0],
              [0.5, 0.4, 0],
              [0.5, -0.4, 0]
          ]
        )
  )
)
print(generator.constraints.tags)