#!/bin/bash
python3 -m virtualenv frontier_exploration_with_a_prior
source ./frontier_exploration_with_a_prior/bin/activate
pip3 install -r requirements.txt
export PYTHONPATH=$PYTHONPATH:$PWD/preprocessing

