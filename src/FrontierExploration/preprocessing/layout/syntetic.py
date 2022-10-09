import random
from typing import Optional
from shapely.geometry import LineString, Point
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
        output_dir: str,
        n_rectangles: int,
        n_in_rectangles: int = None,
        n_points: int = None,
        x_room_range: int = 50,
        y_room_range: int = 50,
        wall_thickness: float = 0.1,
        wall_height: float = 5
    ):    
        self.world_name = name
        self.n_rectangles = n_rectangles
        self.n_internal_rectangles = n_in_rectangles
        self.n_points = n_points
        self.x_room_range = x_room_range
        self.y_room_range = y_room_range
        self.wall_thickness = wall_thickness
        self.wall_height = wall_height
        self.export_models_dir = f"{output_dir}/models"
        self.export_world_dir = f"{output_dir}/worlds"
        self.wall_polygon = self.create_polygon()
        self._starting_point = None
        # Create a world generator to place
        # objects in the world
        self.world_generator = WorldGenerator()

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
        elif self.n_points is not None:
            if self.n_points < 3:
                raise ValueError(
                    'Number of points for triangulation must be at least 3,'
                    ' provided={}'.format(self.n_points))
            wall_polygon = random_points_to_triangulation(
                self.n_points)
        else:
            raise ValueError(
                'No number of rectangles and no number of points for'
                ' triangulation were provided')
        if self.n_internal_rectangles is not None and self.n_internal_rectangles > 0:
            internal_polygon = None
            while(internal_polygon is None or wall_polygon.contains(internal_polygon)):
                if self.n_internal_rectangles > 1:
                    internal_polygon = random_rectangles(
                            n_rect=self.n_internal_rectangles,
                            x_center_min=-self.x_room_range / 2.,
                            x_center_max=self.x_room_range / 2.,
                            y_center_min=-self.y_room_range / 2.,
                            y_center_max=self.y_room_range / 2.,
                            delta_x_min=self.x_room_range / 4.,
                            delta_x_max=self.x_room_range / 2,
                            delta_y_min=self.y_room_range / 4.,
                            delta_y_max=self.y_room_range / 2)
                elif self.n_internal_rectangles == 1:
                    internal_polygon = random_rectangle(
                            delta_x_min=self.x_room_range / 4.,
                            delta_x_max=self.x_room_range / 2,
                            delta_y_min=self.y_room_range / 4.,
                            delta_y_max=self.y_room_range / 2)
            
            wall_polygon = wall_polygon.difference(internal_polygon)
        return wall_polygon
    
    def create_world(
        self,
        n_cubes: Optional[int] = None,
        cube_size: Optional[float] = 1,
        show=True):

        self.n_cubes = n_cubes

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



        # Add walls and ground plane to the world
        self.world_generator.world.add_model(
            tag=walls_model.name,
            model=walls_model)
        self.world_generator.world.add_model(
            tag='ground_plane',
            model=SimulationModel.from_gazebo_model('ground_plane'))

        # Retrieve the free space polygon where objects
        # can be placed within the walls
        free_space_polygon = self.world_generator.world.get_free_space_polygon(
            ground_plane_models=[walls_model.name],
            ignore_models=['ground_plane'])

        # Add the workspace constraint to the
        # generator
        self.world_generator.add_constraint(
            name='room_workspace',
            type='workspace',
            frame='world',
            geometry_type='polygon',
            polygon=free_space_polygon
        )

        # Add constraint to place all object
        # tangent to the ground
        self.world_generator.add_constraint(
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
            self.world_generator.add_asset(
                tag='box',
                description=dict(
                    type='box',
                    args=dict(
                        size=f"__import__('numpy').random.uniform(0.1, {cube_size}, 3)",
                        name='cuboid',
                        color='xkcd'
                    )
                )
            )
            models['box'] = self.n_cubes

        if self.n_cylinders is not None and self.n_cylinders > 0:
            self.world_generator.add_asset(
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
            self.world_generator.add_asset(
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
        self.world_generator.add_engine(
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

        # Run placement engine
        self.world_generator.run_engines(attach_models=True)
        self.free_space_polygon = self.world_generator.world.get_free_space_polygon(
            ground_plane_models=self.world_generator.world.models.keys(),
            ignore_models=['ground_plane'])
        self.world_generator.world.name = self.world_name
        if show:
            self.world_generator.world.show()

        add_custom_gazebo_resource_path(self.export_models_dir)

        # Export world to file and walls model as Gazebo model
        full_world_filename = self.world_generator.export_world(
            output_dir=self.export_world_dir,
            filename=self.world_generator.world.name + '.world',
            models_output_dir=self.export_models_dir,
            overwrite=True)

    @property
    def total_area(self):
        return self.free_space_polygon.area
    
    @property
    def starting_point(self):
        if self._starting_point is None:
            self._starting_point = self.get_starting_point()
        return self._starting_point

    def get_border(self, interior: int, exterior: int):
        polygon = self.free_space_polygon
        polygon.exterior.coords
        bound_in = LineString(polygon.exterior.coords).buffer(interior)
        bound_ext = LineString(polygon.exterior.coords).buffer(exterior)
        return bound_ext.intersection(polygon).difference(bound_in)

    def get_starting_point(self, interior: int = 2, exterior: int = 5):
        border = self.get_border(interior, exterior)
        minx, miny, maxx, maxy = border.bounds
        pnt = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        while not border.contains(pnt):
            pnt = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        return pnt
