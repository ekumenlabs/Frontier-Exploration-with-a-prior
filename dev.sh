#!/bin/bash
python -m pip install --upgrade pip venv
python -m venv ./venv
source ./venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e .
