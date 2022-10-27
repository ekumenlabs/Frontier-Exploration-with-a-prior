FROM create_melodic_nvidia

USER root
WORKDIR /create_ws/Frontier-Exploration-with-a-prior/
WORKDIR /create_ws
COPY ./setup.py ./requirements.txt /create_ws/Frontier-Exploration-with-a-prior/
COPY ./src /create_ws/Frontier-Exploration-with-a-prior/src
RUN python3.8 -m pip install --upgrade pip && python3.8 -m pip install /create_ws/Frontier-Exploration-with-a-prior

USER ${USER}
