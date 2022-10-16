#!./venv/bin/python3.8
import typer
from functools import partial
from concurrent.futures import ThreadPoolExecutor, as_completed

from typing import Optional, List
import pandas as pd
from os.path import expanduser

from FrontierExploration.preprocessing.layout.reader import LayoutReader
from FrontierExploration.preprocessing.layout.syntetic import SynteticWorld
from helper.utils import DockerHandler, create_and_save, create_world_model, save_worlds_df


app = typer.Typer(help="Hi Im the Frontier Exploration Scripting Helper!")


@app.command()
def clean_file(
    file: str = typer.Option(
        help="DXF file to clean.", default="test"),
    file_dir: Optional[str] = typer.Option(
        help="Workspace directory.", default="."),
    layers: Optional[List[str]] = typer.Option(
        help="Layers from DXF file to work with.", default=["GAZEBO"]),
    object_types: Optional[List[str]] = typer.Option(
        help="Object types from DXF file to work with. (ex. LINE, LWPOLYLINE, etc.)", default=["LINE", "LWPOLYLINE"]),
    output_file: Optional[str] = typer.Option(
        help="Cleaned file name.", default="clean_file.dxf"), 
    
):
    print("Starting file cleanup...")
    layout_reader = LayoutReader(file_name=file, file_extension="dxf", files_dir=f"{file_dir}/")
    layout_reader.create_clean_dxf(f"{file_dir}/{output_file}", layers, object_types)


@app.command()
def create_world(
    file: str = typer.Option(
        help="DXF file to use for .world creation.", default="test"),
    file_dir: Optional[str] = typer.Option(
        help="Workspace directory.", default="."),
    output_file_dir: Optional[str] = typer.Option(
        help="Directory to save Gazebo files.", default="/home/.gazebo"),
    show: Optional[bool] = typer.Option(
        help="Flag for showing Gazebo world after creation.", default=False),
):
    print("Starting world creation...")
    layout_reader = LayoutReader(file_name=file, file_extension="dxf", files_dir=f"{file_dir}/")
    layout_reader.create_gazebo_model(output_file_dir=output_file_dir, show=show)

@app.command()
def create_world_from_df(
    file: str = typer.Option(
        help="DataFrame file to use for .world creation.", default="worlds_df.pkl"),
    file_dir: Optional[str] = typer.Option(
        help="Workspace directory.", default="."),
    output_file_dir: Optional[str] = typer.Option(
        help="Directory to save Gazebo files.", default=expanduser("~/.gazebo")),
    cubes:  Optional[int] = typer.Option(
        help="Random cubes to add inside world.", default=0),
    cube_size:  Optional[float] = typer.Option(
        help="Random cubes max size.", default=10),
    show: Optional[bool] = typer.Option(
        help="Flag for showing Gazebo world after creation.", default=False),
):
    print("Starting world creation...")
    worlds = {}
    worlds_df = pd.read_pickle(f"{file_dir}/{file}")
    create_world_model_partial = partial(
        create_world_model,
        cubes=cubes,
        cube_size=cube_size,
        show=show
    )
    with ThreadPoolExecutor(max_workers=10) as executor:
        futures = [executor.submit(create_world_model_partial, file=index, world=worlds_df.loc[index]["world"]) for index in worlds_df.index]
        for future in as_completed(futures):
            res = future.result()
            if res is None:
                print(f"Failed to create world, skipping.")
                continue

            name, world = res
            print(f"Created world {name}")
            worlds[name] = {
                "world":world,
                "world_dir":f"{output_file_dir}worlds/{name}.world"
            }
    print(worlds)
    save_worlds_df(worlds, output_file_dir=output_file_dir, create_models=True)

@app.command()
def create_random(
    file: str = typer.Option(
        help="DXF file to use for .world creation.", default="test"),   
    output_file_dir: Optional[str] = typer.Option(
        help="Directory to save Gazebo files.", default="/home/.gazebo"),
    show: Optional[bool] = typer.Option(
        help="Flag for showing Gazebo world after creation.", default=False),
    rectangles: Optional[int] = typer.Option(
        help="Rectangles used in world creation.", default=None),
    interior_rectangles: Optional[int] = typer.Option(
        help="Interior rectangles used in world creation.", default=None),
    points: Optional[int] = typer.Option(
        help="Points used in world creation (rectangles value has higher priority).", default=None),
    x_range: Optional[int] = typer.Option(
        help="Range in X axis.", default=50),
    y_range: Optional[int] = typer.Option(
        help="Range in Y axis.", default=50),
    wall_thickness: Optional[float] = typer.Option(
        help="Walls thickness.", default=0.1),
    wall_height: Optional[float] = typer.Option(
        help="Walls height.", default=5),
    cubes:  Optional[int] = typer.Option(
        help="Random cubes to add inside world.", default=0),
    cube_size:  Optional[float] = typer.Option(
        help="Random cubes max size.", default=10),
    n_worlds: Optional[int] = typer.Option(
        help="Amount of worlds to create.", default=1),
    create_models: Optional[bool] = typer.Option(
        help="Flag that indicates if the worlds should be created or just the world polygon and df.", default=True),
):
    worlds = {}
    if n_worlds == 1:
        print("Starting single random world creation...")
        world = SynteticWorld(
            file,
            output_file_dir,
            n_rectangles=rectangles,
            n_in_rectangles=interior_rectangles,
            x_room_range=x_range,
            y_room_range=y_range,
            wall_thickness=wall_thickness,
            wall_height=wall_height
        )
        if create_models:
            world.create_world(n_cubes=cubes, cube_size=cube_size, show=show)
        worlds[file] = {
                    "world":world,
                    "world_dir":f"{output_file_dir}worlds/{file}.world"
                }
    elif n_worlds > 1:
        print("Starting multiple random world creation...")
        create_and_save_partial = partial(
            create_and_save,
            file=file,
            output_file_dir=output_file_dir,
            rectangles=rectangles,
            n_in_rectangles=interior_rectangles,
            x_range=x_range,
            y_range=y_range,
            wall_thickness=wall_thickness,
            wall_height=wall_height,
            cubes=cubes,
            cube_size=cube_size,
            show=show,
            create_models=create_models
        )
        with ThreadPoolExecutor(max_workers=10) as executor:
            ids = [id for id in range(n_worlds)]
            futures = [executor.submit(create_and_save_partial, id=id) for id in ids]
            for future in as_completed(futures):
                res = future.result()
                if res is None:
                    print(f"Failed to create world, skipping.")
                    continue

                name, world = res
                print(f"Created world {name}")
                worlds[name] = {
                    "world":world,
                    "world_dir":f"{output_file_dir}worlds/{name}.world"
                }
                save_worlds_df(worlds, output_file_dir=output_file_dir, create_models=create_models)


@app.command()
def run(
    worlds_df_path: str = typer.Option(
        help="Dataframe to use for Docker instance creations.", default="worlds_df.pkl"),
    models_path: str = typer.Option(
        help="Models directory path where worlds models are stored.", default=None),
    world_name: str = typer.Option(
        help="Models directory path where worlds models are stored.", default=None),
    visibility: str = typer.Option(
        help="Models directory path where worlds models are stored.", default="false"),

):
    if visibility not in ("false", "true"):
        raise RuntimeError(f"Visibility has to be either 'false' or 'true'")
    DockerHandler(worlds_df_path, models_path=models_path).run(world_name=world_name, visibility="false")

@app.command()
def run_docker(
    worlds_df_path: str = typer.Option(
        help="Dataframe to use for Docker instance creations.", default="worlds_df.pkl"),
    models_path: str = typer.Option(
        help="Models directory path where worlds models are stored.", default=None)
):
    DockerHandler(worlds_df_path, models_path=models_path).run_all()


if __name__ == "__main__":
    app()