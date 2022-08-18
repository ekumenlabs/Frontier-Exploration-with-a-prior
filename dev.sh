#!/bin/bash
python3.8 -m pip install --upgrade pip venv
python3.8 -m venv ./venv
source ./venv/bin/activate
python3.8 -m pip install -r requirements.txt
python3.8 -m pip install -e .
