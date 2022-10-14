import subprocess
import os
import pickle
import random
from time import sleep
from typing import Any, Dict

import pandas as pd

from FrontierExploration.preprocessing.layout.syntetic import SynteticWorld
from ROS.create_autonomy.docker import docker_utils as ut
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm

def create_and_save(
    file: str,
    output_file_dir: str,
    rectangles: int,
    n_in_rectangles: int,
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
        n_in_rectangles=n_in_rectangles,
        x_room_range=x_range,
        y_room_range=y_range,
        wall_thickness=wall_thickness,
        wall_height=wall_height
    )
    try:
        world.create_world(n_cubes=random.randint(0, cubes), cube_size=cube_size, show=show)
    except Exception as e:
        print(e)
        return None
    return file, world

def save_worlds_df(worlds: Dict[str, Any], output_file_dir: str):
    worlds_df = pd.DataFrame(worlds).T
    worlds_df["starting_point"] = worlds_df["world"].apply(lambda x: x.starting_point.coords[0])
    worlds_df["free_area"] = worlds_df["world"].apply(lambda x: x.free_space_polygon.area)
    with open(f"{output_file_dir}/worlds_df.pkl", "wb") as f:
        pickle.dump(worlds_df, f)


class DockerHandler:

    IMAGE_NAME = "robotica_utn_frba"
    WS_NAME    = "create_ws"
    UID        = ut.get_uid()

    def __init__(self, worlds_df_path: str, models_path: str = None):
        self._worlds_df = None
        self.worlds_df_path = worlds_df_path
        self.models_path = models_path

    def run(self, world_name:str, visibility:str):
        command = self.get_command(world_name, visibility)
        name = self.get_name(world_name, visibility)
        self.run_dev_environment(name=name, command=command)

    def run_all(self):
        futures = []
        i = 0
        with ThreadPoolExecutor(max_workers=3) as pool:
            for world in self.worlds_df.index:
                i+=1
                futures.append(pool.submit(self.run, world, "false"))
                futures.append(pool.submit(self.run, world, "true"))
                if i > 10:
                    break
            for fut in tqdm(as_completed(futures)):
                fut.result()

    def run_dev_environment(self, name, command="bash", ros="melodic", gazebo="9"):
        user_docker = "create"
        docker_home = f"/home/{user_docker}"
        dockerfile  = f"create_ros_{ros}_gazebo{gazebo}"
        temp_volume = f"$HOME/.{self.IMAGE_NAME}"

        docker_args = []
        docker_args.append("-it")
        docker_args.append("--rm")
        docker_args.append("--env DISPLAY")
        docker_args.append("--volume /tmp/.X11-unix:/tmp/.X11-unix:rw")
        docker_args.append(f"--volume $HOME/.Xauthority:{docker_home}/.Xauthority:rw")
        docker_args.append(f"--volume $HOME/.bash_history:{docker_home}/.bash_history:rw")
        docker_args.append(f"--name {name}")
        docker_args.append("--privileged")
        # docker_args.append(f"--network algo{self.UID}")
        docker_args.append(f"--user {self.UID}:{self.UID}")
        if self.models_path is not None:
            docker_args.append(f"--volume {self.models_path}:{self.models_path}")
        # Keep user settings
        docker_args.append(f"--volume $HOME/.gazebo/:{docker_home}/.gazebo/")
        # docker_args.append(f"--volume {temp_volume}/.gazebo/:{docker_home}/.gazebo/")
        # Mount workspace
        docker_args.append(f"--volume {os.path.dirname(ut.get_repo_root())}/Frontier-Exploration-with-a-prior/src/ROS/create_autonomy:/{self.WS_NAME}/src")
        docker_args.append(f"--volume {os.path.dirname(ut.get_repo_root())}/Frontier-Exploration-with-a-prior:/{self.WS_NAME}/Frontier-Exploration-with-a-prior")
        docker_args.append(f"--volume {temp_volume}/ws/build/:/{self.WS_NAME}/build/")
        docker_args.append(f"--volume {temp_volume}/ws/devel/:/{self.WS_NAME}/devel/")
        docker_args.append(f"--volume $HOME/utn_worlds/:{docker_home}/utn_worlds/")
        # VSCode needs HOME to be defined in order to work in the container
        docker_args.append(f"-e HOME={docker_home}")

        # USB devices for the real robot
        docker_args.append(ut.mount_resource("/dev/roomba"))
        docker_args.append(ut.mount_resource("/dev/rplidar"))
        docker_args.append(ut.mount_resource("/dev/video0"))
        docker_args.append(ut.mount_resource("/dev/video1"))
        docker_args.append(ut.mount_resource("/dev/video2"))
        docker_args.append(ut.mount_resource("/dev/video3"))
        docker_args.append(ut.mount_resource("/dev/video4"))
        docker_args.append(ut.mount_resource("/dev/video5"))
        docker_args.append(ut.mount_resource("/dev/video6"))
        docker_args.append(ut.mount_resource("/dev/video7"))
        docker_args.append(ut.mount_resource("/dev/video8"))
        docker_args.append(ut.mount_resource("/dev/video9"))

        # To allow installing packages
        docker_args.append("--group-add=sudo")

        docker_args.append("-e ROS_HOSTNAME=localhost")
        docker_args.append("-e ROS_MASTER_URI=http://localhost:11311")
        docker_args.append(f"--workdir /{self.WS_NAME}/")
        # run dettached
        # docker_args.append(f"-d")

        if ut.is_nvidia():
            docker_args.append("--gpus all")
            dockerfile = f"create_{ros}_nvidia"

        # Join arguments together separated by a space
        docker_args = ' '.join(docker_args)
        docker_command = f"docker run {docker_args} {dockerfile} {command}"

        ut.create_directory(f"{temp_volume}/.gazebo/")
        ut.create_directory(f"{temp_volume}/ws/build/")
        ut.create_directory(f"{temp_volume}/ws/devel/")

        ut.run_command("xhost +local:root")
        ut.run_command(docker_command)
        ut.run_command("xhost -local:root")


    def attach_dev_environment(self, command="bash"):
        command = f"docker exec -it --user {self.UID}:{self.UID} {self.IMAGE_NAME} {command}"
        ut.run_command(command)


    def is_running(self):
        command = f"docker ps | grep {self.IMAGE_NAME} > /dev/null"
        try:
            subprocess.check_call(command, shell=True)
        except Exception:
            return False

        return True

    @property
    def worlds_df(self):
        if self._worlds_df is None:
            with open(self.worlds_df_path, "rb") as f:
                self._worlds_df = pickle.load(f)
        return self._worlds_df

    def get_command(self, world_name: str, visibility: bool):
        row = self.worlds_df.loc[world_name]
        start_x, start_y = row["starting_point"]
        return f"./src/run_exploration.sh {start_x} {start_y} {world_name} {visibility}"

    def get_name(self, world_name: str, visibility: bool):
        return f"{world_name}_{visibility}"
