import json
import os


def get_points(file_path):
    extra_pts = []

    for pt in open(file_path).readlines():
        extra_pts.append(list(map(float, pt.strip('\n').split(' '))))
    return extra_pts


def write_extra_points(folder_path, point_dict):
    file_path = os.path.join(folder_path, 'extra_points.json')
    f = open(file_path, 'w+')
    f.writelines(json.dumps(point_dict))


def load_extra_pts(folder_path, axle):
    folder_path = os.path.join(folder_path, 'extra_points')
    extra_points = {'far': {'flange_right': {'up': None, 'down': None}}}
    axle_map = {'1': 'st', '2': 'nd', '3': 'rd'}

    axle_folder_name = axle_map[axle] if axle in axle_map.keys() else 'th'
    file_path = os.path.join(folder_path, axle, axle + axle_folder_name + '_axle_FlangeRightFar_down_3dpts.txt')
    extra_points['far']['flange_right']['down'] = get_points(file_path)

    file_path = os.path.join(folder_path, axle, axle + axle_folder_name + '_axle_FlangeRightFar_up_3dpts.txt')
    extra_points['far']['flange_right']['up'] = get_points(file_path)

    # write_extra_points(os.path.dirname(folder_path), extra_points)
    return extra_points


if __name__ == '__main__':
    load_extra_pts(r'D:\Project_Data\Field_data\21-02-2022_13-53-04\Data', [1, 5, 6, 7, 8, 9, 10])
