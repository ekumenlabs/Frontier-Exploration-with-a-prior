#!./venv/bin/python3
import typer

from typing import Optional, List

from FrontierExploration.preprocessing.layout.reader import LayoutReader
from FrontierExploration.preprocessing.layout.syntetic import SynteticWorld

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
        help="Random cubes to add inside world.", default=None),
    cube_size:  Optional[float] = typer.Option(
        help="Random cubes max size.", default=None),
):
    print("Starting random world creation...")
    syntetic_world = SynteticWorld(
        name=file,
        output_dir=output_file_dir,
        n_rectangles=rectangles,
        n_in_rectangles=interior_rectangles,
        n_points=points,
        x_room_range=x_range,
        y_room_range=y_range,
        wall_thickness=wall_thickness,
        wall_height=wall_height
    )
    syntetic_world.create_world(n_cubes=cubes, cube_size=cube_size, show=show)


if __name__ == "__main__":
    app()