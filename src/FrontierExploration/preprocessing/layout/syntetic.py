import os
import argparse
import datetime
from pcg_gazebo.simulation import SimulationModel, \
    add_custom_gazebo_resource_path
from pcg_gazebo.generators.creators import extrude
from pcg_gazebo.generators.shapes import random_rectangles, \
    random_rectangle, random_points_to_triangulation
from pcg_gazebo.generators import WorldGenerator


class SynteticWorld:
    def __init__(
        self, 
        name: str, 
        n_rectangles: int, 
        x_room_range: int = 50,
        y_room_range: int = 50,
        wall_thickness: float = 0.1,
        wall_height: float = 5
    ):    
        self.world_name = name
        self.n_rectangles = n_rectangles
        self.x_room_range = x_room_range
        self.y_room_range = y_room_range
        self.wall_thickness = wall_thickness
        self.wall_height = wall_height
        self.export_models_dir = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
        self.export_world_dir = os.path.join(os.path.expanduser('~'), '.gazebo', 'worlds')
        self.wall_polygon = self.create_polygon()

        self.n_cubes = None
        self.n_cylinders = None
        self.n_spheres = None
        self.set_random_roll = False
        self.set_random_pitch = False

    def create_polygon(self):
        # Generate the reference polygon for the wall boundaries
        if self.n_rectangles is not None:
            if self.n_rectangles > 1:
                wall_polygon = random_rectangles(
                    n_rect=self.n_rectangles,
                    x_center_min=-self.x_room_range / 2.,
                    x_center_max=self.x_room_range / 2.,
                    y_center_min=-self.y_room_range / 2.,
                    y_center_max=self.y_room_range / 2.,
                    delta_x_min=self.x_room_range / 2.,
                    delta_x_max=self.x_room_range,
                    delta_y_min=self.y_room_range / 2.,
                    delta_y_max=self.y_room_range)
            elif self.n_rectangles == 1:
                wall_polygon = random_rectangle(
                    delta_x_min=self.x_room_range / 2.,
                    delta_x_max=self.x_room_range,
                    delta_y_min=self.y_room_range / 2.,
                    delta_y_max=self.y_room_range)
            else:
                raise ValueError(
                    'Number of rectangles is invalid, provided={}'.format(
                       self.n_rectangles))
        elif n_points is not None:
            if n_points < 3:
                raise ValueError(
                    'Number of points for triangulation must be at least 3,'
                    ' provided={}'.format(n_points))
            wall_polygon = random_points_to_triangulation(
                n_points)
        else:
            raise ValueError(
                'No number of rectangles and no number of points for'
                ' triangulation were provided')
        return wall_polygon
    
    def create_world(self):
        # Create the wall model based on the extruded
        # boundaries of the polygon
        walls_model = extrude(
            polygon=self.wall_polygon,
            thickness=self.wall_thickness,
            height=self.wall_height,
            pose=[0, 0, self.wall_height / 2., 0, 0, 0],
            extrude_boundaries=True,
            color='xkcd')
        walls_model.name = self.world_name + '_walls'

        # Create a world generator to place
        # objects in the world
        world_generator = WorldGenerator()

        # Add walls and ground plane to the world
        world_generator.world.add_model(
            tag=walls_model.name,
            model=walls_model)
        world_generator.world.add_model(
            tag='ground_plane',
            model=SimulationModel.from_gazebo_model('ground_plane'))

        # Retrieve the free space polygon where objects
        # can be placed within the walls
        free_space_polygon = world_generator.world.get_free_space_polygon(
            ground_plane_models=[walls_model.name],
            ignore_models=['ground_plane'])

        # Add the workspace constraint to the
        # generator
        world_generator.add_constraint(
            name='room_workspace',
            type='workspace',
            frame='world',
            geometry_type='polygon',
            polygon=free_space_polygon
        )

        # Add constraint to place all object
        # tangent to the ground
        world_generator.add_constraint(
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

        # Add assets
        models = dict()
        if self.n_cubes is not None and self.n_cubes > 0:
            world_generator.add_asset(
                tag='box',
                description=dict(
                    type='box',
                    args=dict(
                        size="__import__('numpy').random.uniform(0.1, 1, 3)",
                        name='cuboid',
                        color='xkcd'
                    )
                )
            )
            models['box'] = self.n_cubes

        if self.n_cylinders is not None and self.n_cylinders > 0:
            world_generator.add_asset(
                tag='cylinder',
                description=dict(
                    type='cylinder',
                    args=dict(
                        radius="__import__('numpy').random.uniform(0.1, 1)",
                        length="__import__('numpy').random.uniform(0.1, 1)",
                        name='cylinder',
                        color='xkcd'
                    )
                )
            )
            models['cylinder'] = self.n_cylinders

        if self.n_spheres is not None and self.n_spheres > 0:
            world_generator.add_asset(
                tag='sphere',
                description=dict(
                    type='sphere',
                    args=dict(
                        radius="__import__('numpy').random.uniform(0.1, 1)",
                        name='sphere',
                        color='xkcd'
                    )
                )
            )
            models['sphere'] = self.n_spheres

        orientation_dofs = ['yaw']
        if self.set_random_roll:
            orientation_dofs.append('roll')
        if self.set_random_pitch:
            orientation_dofs.append('pitch')

        # Add placement policy
        placement_policy = dict(
            models=list(models.keys()),
            config=[
                dict(
                    dofs=['x', 'y'],
                    tag='workspace',
                    workspace='room_workspace'
                ),
                dict(
                    dofs=orientation_dofs,
                    tag='uniform',
                    mean=0,
                    min=-3.141592653589793,
                    max=3.141592653589793
                )
            ]
        )

        # Set local constraints
        local_constraints = list()
        for m in models:
            local_constraints.append(
                dict(model=m, constraint='tangent_to_ground_plane'))

        # Place objects randomly on the free
        # space within the walls
        world_generator.add_engine(
            tag='placement_engine',
            engine_name='random_pose',
            models=list(models.keys()),
            max_num=models,
            model_picker='random',
            no_collision=True,
            min_distance=0.0,
            policies=[placement_policy],
            constraints=local_constraints
        )

        # # Run placement engine
        # world_generator.run_engines(attach_models=True)
        world_generator.world.name = self.world_name
        # if preview:
        world_generator.world.show()

        add_custom_gazebo_resource_path(self.export_models_dir)

        # Export world to file and walls model as Gazebo model
        full_world_filename = world_generator.export_world(
            output_dir=self.export_world_dir,
            filename=world_generator.world.name + '.world',
            models_output_dir=self.export_models_dir,
            overwrite=True)
