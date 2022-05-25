import json
import math
import os

import open3d as o3d

from draw_graphs import *
from load_data import AxleData, laser_dict, read_axle_data
from load_extra_points import load_extra_pts


def get_line_eq_2pts(p1, p2):
    line_eq = [(p2[0] - p1[0]), (p2[1] - p1[1]), (p2[2] - p1[2])]
    return np.divide(line_eq, np.linalg.norm(line_eq))


def get_plane_eq_3pts(p1, p2, p3):
    a1 = p2[0] - p1[0]
    b1 = p2[1] - p1[1]
    c1 = p2[2] - p1[2]
    a2 = p3[0] - p1[0]
    b2 = p3[1] - p1[1]
    c2 = p3[2] - p1[2]
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    # print("equation of plane is ", a, "x +", b, "y +", c, "z = ", d)
    plane_eq = [a, b, c]
    norm_plane_eq = np.divide(plane_eq, np.linalg.norm(plane_eq))

    d = (norm_plane_eq[0] * p1[0] + norm_plane_eq[1] * p1[1] + norm_plane_eq[2] * p1[2])

    return np.concatenate((norm_plane_eq, [d]), axis=0)


def get_pt_projection_3dline(pt, line_eq=None):
    p = np.array([pt[0], pt[1], pt[2]])
    # a = np.array([pt_line.x, pt_line.y, pt_line.z])
    u = np.array(line_eq)

    proj_p = p.dot(u) * u
    return proj_p


def get_plane_eq_pt_line(pt, line_eq):
    d = np.array(pt).dot(-1 * np.array(line_eq))
    return list(line_eq) + [d]


def point_projection_on_line_given_line_eqn(line_normal, line_point, pt):
    a = line_normal[0]
    b = line_normal[1]
    c = line_normal[2]
    x_bias = line_point[0]
    y_bias = line_point[1]
    z_bias = line_point[2]
    denom = (a * a) + (b * b) + (c * c)
    t = ((a * (pt[0] - x_bias)) + (b * (pt[1] - y_bias)) + (c * (pt[2] - z_bias))) / denom
    corres_cx = (a * t) + x_bias
    corres_cy = (b * t) + y_bias
    corres_cz = (c * t) + z_bias

    return corres_cx, corres_cy, corres_cz


def get_2dpoints(normal_plane, profile_points):
    points_2d = []
    for point in profile_points:
        pt = list(map(float, point))
        proj_xp, proj_yp, proj_zp = point_projection_on_line_given_line_eqn(normal_plane[:3], normal_plane[4:], pt)

        y2d = np.sqrt(math.pow(proj_xp - pt[0], 2) + math.pow(proj_yp - pt[1], 2) + math.pow(proj_zp - pt[2], 2))
        x2d = (normal_plane[0] * pt[0]) + (normal_plane[1] * pt[1]) + (normal_plane[2] * pt[2]) + normal_plane[3]

        points_2d.append([x2d, y2d])
    return np.array(points_2d), points_2d


def get_2dpoints_op2(center_axis_eq_pts, normal_plane, profile_points, fwall_dist, twall_dist, cam_side):
    points_2d = []
    for point in profile_points:
        pt = list(map(float, point))
        proj_xp, proj_yp, proj_zp = point_projection_on_line_given_line_eqn(center_axis_eq_pts[:3],
                                                                            center_axis_eq_pts[4:], pt)

        y2d = np.sqrt(math.pow(proj_xp - pt[0], 2) + math.pow(proj_yp - pt[1], 2) + math.pow(proj_zp - pt[2], 2))
        x2d = (normal_plane[0] * pt[0]) + (normal_plane[1] * pt[1]) + (normal_plane[2] * pt[2]) + normal_plane[3]
        if cam_side == 'f':
            if twall_dist < 0:
                points_2d.append([-x2d, y2d])
            else:
                points_2d.append([x2d, y2d])
        else:
            if fwall_dist < 0:
                points_2d.append([x2d, y2d])
            else:
                points_2d.append([-x2d, y2d])
        points_2d.append([x2d, y2d])
    return np.array(points_2d), points_2d


def get_2dpoints_code_based(center_axis_eq_pts, normal_plane, profile_points, fwall_dist, twall_dist, cam_side):
    points_2d = []
    for point in profile_points:
        pt = list(map(float, point))
        proj_xp, proj_yp, proj_zp = point_projection_on_line_given_line_eqn(center_axis_eq_pts[:3],
                                                                            center_axis_eq_pts[4:], pt)
        y2d = np.sqrt(math.pow(proj_xp - pt[0], 2) + math.pow(proj_yp - pt[1], 2) + math.pow(proj_zp - pt[2], 2))
        x2d = (normal_plane[0] * pt[0]) + (normal_plane[1] * pt[1]) + (normal_plane[2] * pt[2]) + normal_plane[3]
        if cam_side == 'f':
            if twall_dist < 0:
                points_2d.append([-x2d, y2d])
            else:
                points_2d.append([x2d, y2d])
        else:
            if fwall_dist < 0:
                points_2d.append([abs(twall_dist) + x2d, y2d])
            else:
                points_2d.append([abs(twall_dist) - x2d, y2d])
    return np.array(points_2d), points_2d


def write_2d_points_to_file_json(fprofiles, tprofiles, axle, save_dir):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for k in fprofiles:
        profile_dict = {'flange_points': list(fprofiles[k]), 'tread_points': list(tprofiles[k])}
        file_name = axle + '_laser_profile_' + str(k) + '.json'
        f = open(os.path.join(save_dir + file_name), 'w+')
        f.writelines(json.dumps(profile_dict))


def write_2d_points_to_file(fprofiles, tprofiles, axle_id, save_dir):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for k in fprofiles:
        ffile_name = axle_id + '_flange_laser_profile_' + str(k) + '.ply'
        fpcd = o3d.geometry.PointCloud()
        fpcd.points = o3d.utility.Vector3dVector(np.array([x + [0] for x in fprofiles[k]]))
        o3d.io.write_point_cloud(os.path.join(save_dir + ffile_name), fpcd)

        tfile_name = axle_id + '_tread_laser_profile_' + str(k) + '.ply'
        tpcd = o3d.geometry.PointCloud()
        tpcd.points = o3d.utility.Vector3dVector(np.array([x + [0] for x in tprofiles[k]]))
        o3d.io.write_point_cloud(os.path.join(save_dir + tfile_name), tpcd)


def prepare_master_dict(fdict: dict, tdict: dict):
    mdict = {}
    for k in fdict:
        mdict.update({k: np.concatenate((fdict[k], tdict[k]), axis=0)})
    return mdict


def get_profiles(main_folder, axle):
    extra_points = load_extra_pts(main_folder, axle)
    fr_3dpts = extra_points['far']['flange_right']

    axle_data = AxleData()
    axle_data = read_axle_data(os.path.join(main_folder, 'Data.json'), int(axle))

    axle_data.flange_profile3d['right_down'] = np.concatenate(
        (np.array(fr_3dpts['down']), axle_data.flange_profile3d['right_down']), axis=0)
    axle_data.flange_profile3d['right_up'] = np.concatenate(
        (np.array(fr_3dpts['up']), axle_data.flange_profile3d['right_up']), axis=0)

    master_dict = {'orginal': laser_dict.copy(), 'calculated': laser_dict.copy()}
    flange_pts_2d = laser_dict.copy()
    tread_pts_2d = laser_dict.copy()

    flange_wall_dist = axle_data.tread_center_axis_eq['a'] * axle_data.flange_wall_refpt[0] + \
                       axle_data.tread_center_axis_eq['b'] * axle_data.flange_wall_refpt[1] + \
                       axle_data.tread_center_axis_eq['c'] * axle_data.flange_wall_refpt[2] + \
                       axle_data.tread_center_axis_eq['d']

    tread_wall_dist = axle_data.flange_center_axis_eq['a'] * axle_data.tread_wall_refpt[0] + \
                      axle_data.flange_center_axis_eq['b'] * axle_data.tread_wall_refpt[1] + \
                      axle_data.flange_center_axis_eq['c'] * axle_data.tread_wall_refpt[2] + \
                      axle_data.flange_center_axis_eq['d']

    # ref_wall_plane = cal_norm_plane
    # get normal plane
    # tread_center_axis_params = [axle_data.tread_center_axis_eq['a'], axle_data.tread_center_axis_eq['b'],
    #                             axle_data.tread_center_axis_eq['c']]
    # ref_pt = get_pt_projection_3dline(axle_data.flange_profile3d['right_up'][10], tread_center_axis_params)
    # cal_norm_plane = get_plane_eq_pt_line(ref_pt, tread_center_axis_params)
    # print(cal_norm_plane)

    ref_wall_plane = list(axle_data.tread_center_axis_eq.values())

    flist = laser_dict.copy()
    tlist = laser_dict.copy()

    for k in flange_pts_2d:
        # flange_pts_2d[k], flist[k] = get_2dpoints(ref_wall_plane, axle_data.flange_profile3d[k])
        # tread_pts_2d[k], tlist[k] = get_2dpoints(ref_wall_plane, axle_data.tread_profile3d[k])

        flange_pts_2d[k], flist[k] = get_2dpoints_op2(ref_wall_plane,
                                                      list(axle_data.flange_center_axis_eq.values()),
                                                      axle_data.flange_profile3d[k], flange_wall_dist,
                                                      tread_wall_dist, 'f')
        tread_pts_2d[k], tlist[k] = get_2dpoints_op2(ref_wall_plane,
                                                     list(axle_data.flange_center_axis_eq.values()),
                                                     axle_data.tread_profile3d[k], flange_wall_dist,
                                                     tread_wall_dist, 't')
        # flange_pts_2d[k], flist[k] = get_2dpoints_code_based(ref_wall_plane,
        #                                                      list(axle_data.flange_center_axis_eq.values()),
        #                                                      axle_data.flange_profile3d[k], flange_wall_dist,
        #                                                      tread_wall_dist, 'f')
        # tread_pts_2d[k], tlist[k] = get_2dpoints_code_based(ref_wall_plane,
        #                                                     list(axle_data.tread_center_axis_eq.values()),
        #                                                     axle_data.tread_profile3d[k], flange_wall_dist,
        #                                                     tread_wall_dist, 't')

    master_dict['orginal'] = prepare_master_dict(axle_data.flange_profile2d, axle_data.tread_profile2d)
    master_dict['calculated'] = prepare_master_dict(flange_pts_2d, tread_pts_2d)

    write_2d_points_to_file_json(flist, tlist, axle, os.path.join(main_folder, '2d_points/cal/'))
    write_2d_points_to_file(flist, tlist, axle, os.path.join(main_folder, '2d_points/cal/'))
    # write_2d_points_to_file(flist, tlist, axle, r'D:\Project_Data\Field_data\21-02-2022_13-53-04\Data\2d_points/org/')

    # display_profiles_sep('axle ' + axle + ' 3d', axle_data.flange_profile3d, axle_data.tread_profile3d, is_3d=True)

    display_profiles_overlapping('given vs calculated', master_dict['orginal'], master_dict['calculated'],
                                 'from data.json', 'calculated')
    # display_profiles_sep('axle ' + axle + ' 2d given', axle_data.flange_profile2d, axle_data.tread_profile2d,
    #                      is_3d=False)
    # display_profiles_sep('axle ' + axle + ' 2d calc', flange_pts_2d, tread_pts_2d, is_3d=False)


if __name__ == '__main__':
    main_folder = r'D:\Project_Data\Field_data\04022022\Data'
    get_profiles(main_folder, '6')

    # to_continue = True
    # while to_continue:
    #     axle_id = input("\nEnter axle id: ")
    #     get_profiles(main_folder, axle_id)
    #     to_continue = input("Do you want to try for another axel (y/n) :").lower() == 'y'
