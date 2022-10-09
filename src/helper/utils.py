import pickle
import random
from typing import Any, Dict

import pandas as pd

from FrontierExploration.preprocessing.layout.syntetic import SynteticWorld


def create_and_save(
    file: str,
    output_file_dir: str,
    rectangles: int,
    x_range: float,
    y_range: float,
    wall_thickness: float,
    wall_height: float,
    cubes: int,
    cube_size: float,
    show: bool,
    id: int,
):
    file = f"{file}_{id}"
    world = SynteticWorld(
        file,
        output_file_dir,
        n_rectangles=random.randint(1, rectangles),
        x_room_range=x_range,
        y_room_range=y_range,
        wall_thickness=wall_thickness,
        wall_height=wall_height
    )
    world.create_world(n_cubes=random.randint(0, cubes), cube_size=cube_size, show=show)
    return file, world

def save_worlds_df(worlds: Dict[str, Any], output_file_dir: str):
    worlds_df = pd.DataFrame(worlds).T
    worlds_df["starting_point"] = worlds_df["world"].apply(lambda x: x.starting_point.coords[0])
    worlds_df["free_area"] = worlds_df["world"].apply(lambda x: x.free_space_polygon.area)
    with open(f"{output_file_dir}/worlds_df.pkl", "wb") as f:
        pickle.dump(worlds_df, f)
