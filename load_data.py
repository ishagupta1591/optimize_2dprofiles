import json

import numpy as np
import open3d as o3d

laser_dict = {'left_down': None, 'left_up': None, 'right_down': None, 'right_up': None}
laser_dict_spc = {'right_up': None}


class AxleData:
    def __init__(self):
        self.flange_profile3d = laser_dict.copy()
        self.tread_profile3d = laser_dict.copy()

        self.flange_profile2d = laser_dict.copy()
        self.tread_profile2d = laser_dict.copy()

        self.flange_wallpts3d = laser_dict.copy()
        self.tread_wallpts3d = laser_dict.copy()

        self.flange_wall_refpt = None
        self.tread_wall_refpt = None

        self.tread_center_axis = None
        self.flange_center_axis = None

        self.tread_center_axis_eq = None
        self.flange_center_axis_eq = None

        self.ideal_flange = None
        self.ideal_tread = None


def __get_wall_points(pts, flags):
    wall_pts = np.array([p for p, f in zip(pts, flags) if f])
    sorted_pts = np.array(sorted(wall_pts, key=lambda x: x[0]))
    return sorted_pts


def __get_2dpoints__(laser_dict):
    l1 = np.array(laser_dict['Down'][1]['2d_profile'])
    l2 = np.array(laser_dict['Up'][1]['2d_profile'])
    return l1, l2


def __get_3dpoints__(laser_dict):
    laser1 = laser_dict['Down'][2]['3d_lineflag']
    pts1 = np.array([np.array(x[0], dtype=object) for x in laser1], dtype=object)
    flag1 = np.array([x[1] for x in laser1], dtype=object)
    if any(flag1):
        wallpts1 = __get_wall_points(pts1, flag1)
        ref_pt1 = [np.mean(wallpts1[:, 0]), np.mean(wallpts1[:, 1]), np.mean(wallpts1[:, 2])]
    else:
        raise Exception("No wall points found!")

    laser2 = laser_dict['Up'][2]['3d_lineflag']
    pts2 = np.array([np.array(x[0], dtype=object) for x in laser2], dtype=object)
    flag2 = np.array([x[1] for x in laser2], dtype=object)
    if any(flag2):
        wallpts2 = __get_wall_points(pts2, flag2)
        ref_pt2 = [np.mean(wallpts2[:, 0]), np.mean(wallpts2[:, 1]), np.mean(wallpts2[:, 2])]
    else:
        raise Exception("No wall points found!")

    return pts1, wallpts1, ref_pt1, pts2, wallpts2, ref_pt2


def read_axle_data(data_json_path, axle_id):
    flange_wall_refpts = [0] * 4
    tread_wall_refpts = [0] * 4

    axle_data = AxleData()

    data_file = open(data_json_path, "r")
    master_axle_dict = json.load(data_file)['Axel']
    axle_dict = master_axle_dict[axle_id - 1]
    for key, val in axle_dict.items():
        cam_dict = val[key]['Cam_data']

        flange_left_profile = [v[1] for k, v in cam_dict.items() if 'flange' in k.lower() and 'left' in k.lower()][0]
        tread_left_profile = [v[1] for k, v in cam_dict.items() if 'tread' in k.lower() and 'left' in k.lower()][0]
        flange_right_profile = [v[1] for k, v in cam_dict.items() if 'flange' in k.lower() and 'right' in k.lower()][0]
        tread_right_profile = [v[1] for k, v in cam_dict.items() if 'tread' in k.lower() and 'right' in k.lower()][0]

        axle_data.flange_profile3d['left_down'], axle_data.flange_wallpts3d['left_down'], flange_wall_refpts[0], \
        axle_data.flange_profile3d['left_up'], axle_data.flange_wallpts3d['left_up'], flange_wall_refpts[
            1] = __get_3dpoints__(flange_left_profile)
        axle_data.tread_profile3d['left_down'], axle_data.tread_wallpts3d['left_down'], tread_wall_refpts[0], \
        axle_data.tread_profile3d['left_up'], axle_data.tread_wallpts3d['left_up'], tread_wall_refpts[
            1] = __get_3dpoints__(tread_left_profile)
        axle_data.flange_profile3d['right_down'], axle_data.flange_wallpts3d['right_down'], flange_wall_refpts[2], \
        axle_data.flange_profile3d['right_up'], axle_data.flange_wallpts3d['right_up'], flange_wall_refpts[
            3] = __get_3dpoints__(flange_right_profile)
        axle_data.tread_profile3d['right_down'], axle_data.tread_wallpts3d['right_down'], tread_wall_refpts[2], \
        axle_data.tread_profile3d['right_up'], axle_data.tread_wallpts3d['right_up'], tread_wall_refpts[
            3] = __get_3dpoints__(tread_right_profile)

        axle_data.flange_profile2d['left_down'], axle_data.flange_profile2d['left_up'] = __get_2dpoints__(
            flange_left_profile)
        axle_data.tread_profile2d['left_down'], axle_data.tread_profile2d['left_up'] = __get_2dpoints__(
            tread_left_profile)
        axle_data.flange_profile2d['right_down'], axle_data.flange_profile2d['right_up'] = __get_2dpoints__(
            flange_right_profile)
        axle_data.tread_profile2d['right_down'], axle_data.tread_profile2d['right_up'] = __get_2dpoints__(
            tread_right_profile)

        axle_data.flange_center_axis_eq = val[key]['flange_axis']
        axle_data.tread_center_axis_eq = val[key]['tread_axis']

        axle_data.flange_center_axis = val[key]['flange_centres']
        axle_data.tread_center_axis = val[key]['tread_centres']

        axle_data.ideal_flange = val[key]['ideal_flange']
        axle_data.ideal_tread = val[key]['ideal_tread']

        flange_wall_refpts = np.array(flange_wall_refpts)
        tread_wall_refpts = np.array(tread_wall_refpts)
        axle_data.flange_wall_refpt = [np.mean(flange_wall_refpts[:, 0]), np.mean(flange_wall_refpts[:, 1]),
                                       np.mean(flange_wall_refpts[:, 2])]
        axle_data.tread_wall_refpt = [np.mean(tread_wall_refpts[:, 0]), np.mean(tread_wall_refpts[:, 1]),
                                      np.mean(tread_wall_refpts[:, 2])]

    # if is_3d:
    #     return flange_dict, tread_dict, flange_dict_flag, tread_dict_flag
    # else:
    #     return flange_dict, tread_dict
    return axle_data


def get_vector(laser_file_path):
    laser_file = open(laser_file_path)
    jsonObj = json.load(laser_file)
    flange_pts = jsonObj['flange_points']
    tread_pts = jsonObj['tread_points']

    xyz = np.zeros((len(flange_pts) + len(tread_pts), 3))
    xyz[:len(flange_pts), 0] = [pt[0] for pt in flange_pts]
    xyz[len(flange_pts):len(tread_pts) + len(flange_pts), 0] += [pt[0] for pt in tread_pts]

    xyz[:len(flange_pts), 1] = [pt[1] for pt in flange_pts]
    xyz[len(flange_pts):len(tread_pts) + len(flange_pts), 1] += [pt[1] for pt in tread_pts]
    return xyz


def get_vector_separate(laser_file_path):
    laser_file = open(laser_file_path)
    jsonObj = json.load(laser_file)
    flange_pts = jsonObj['flange_points']
    tread_pts = jsonObj['tread_points']

    fxyz = np.zeros((len(flange_pts), 3))
    txyz = np.zeros((len(tread_pts), 3))

    fxyz[:, 0] = [pt[0] for pt in flange_pts]
    txyz[:, 0] += [pt[0] for pt in tread_pts]

    fxyz[:, 1] = [pt[1] for pt in flange_pts]
    txyz[:, 1] += [pt[1] for pt in tread_pts]
    return fxyz, txyz


def write_ply():
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    laser_1_pcd = o3d.geometry.PointCloud()
    laser_2_pcd = o3d.geometry.PointCloud()
    laser_3_pcd = o3d.geometry.PointCloud()
    laser_4_pcd = o3d.geometry.PointCloud()

    laser_1_pcd.points = o3d.utility.Vector3dVector(laser_1_xyz)
    laser_2_pcd.points = o3d.utility.Vector3dVector(laser_2_xyz)
    laser_3_pcd.points = o3d.utility.Vector3dVector(laser_3_xyz)
    laser_4_pcd.points = o3d.utility.Vector3dVector(laser_4_xyz)

    o3d.io.write_point_cloud("ply_files\\laser1.ply", laser_1_pcd)
    o3d.io.write_point_cloud("ply_files\\laser2.ply", laser_2_pcd)
    o3d.io.write_point_cloud("ply_files\\laser3.ply", laser_3_pcd)
    o3d.io.write_point_cloud("ply_files\\laser4.ply", laser_4_pcd)

    # pcd_load = o3d.io.read_point_cloud("sync.ply")
    # o3d.visualization.draw_geometries([pcd_load])
    #
    # convert Open3D.o3d.geometry.PointCloud to numpy array
    # xyz_load = np.asarray(pcd_load.points)


if __name__ == "__main__":
    # laser1 = r"D:\Projects\cylinder_visualization\2d_points\laser_profile_1.json"
    # laser2 = r"D:\Projects\cylinder_visualization\2d_points\laser_profile_2.json"
    # laser3 = r"D:\Projects\cylinder_visualization\2d_points\laser_profile_3.json"
    # laser4 = r"D:\Projects\cylinder_visualization\2d_points\laser_profile_4.json"
    # laser_1_xyz = get_vector(laser1)
    # laser_2_xyz = get_vector(laser2)
    # laser_3_xyz = get_vector(laser3)
    # laser_4_xyz = get_vector(laser4)
    read_axle_data(r'D:\Project_Data\Field_data\21-02-2022_13-53-04\Data\Data_org.json', 5)
