from pathlib import Path
import json


def read_joints_from_json():
    path = Path().cwd()
    path_json = path / f'robot_api/robot/utils/joints_to_move.json'

    file_json = open(path_json, 'r+')
    file_json = json.load(file_json)

    joints_dict = file_json["move_joints"]

    return joints_dict


def save_to_json():
    # TODO completar
    path = Path().cwd()
    path_json = path / f'robot_api/robot/utils/joints_to_move.json'
