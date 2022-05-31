import os 
from pytest import fixture

@fixture
def files_dir():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    return dir_path + "/tests/files/"

