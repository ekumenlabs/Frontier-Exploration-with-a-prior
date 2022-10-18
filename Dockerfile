FROM create_ros_melodic_gazebo9

USER root
COPY ./setup.py ./requirements.txt /create_ws/Frontier-Exploration-with-a-prior/
COPY ./src /create_ws/Frontier-Exploration-with-a-prior/src
WORKDIR /create_ws/Frontier-Exploration-with-a-prior/
RUN python3.8 -m pip install --upgrade pip && python3.8 -m pip install -e .
WORKDIR /create_ws
USER ${USER}
