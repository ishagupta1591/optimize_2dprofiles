import json
import os

import numpy as np
import plotly.graph_objects as go

from icp import icp
from draw_graphs import display_profiles_overlapping

display_individual = False


def compare_profiles(org_profile, aligned_profiles):
    comp = org_profile == aligned_profiles
    eq_comp = comp.all()
    print(eq_comp)


def write_2d_points_to_file_json(profile, name, save_dir=None, axle_id=None):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    file_name = save_dir + '\\' + str(axle_id) + '_gocator_' + name + '.txt'
    np.savetxt(file_name, profile)


def load_gocator_points(filepath):
    gt_xyz = open(filepath, "r")
    gc_pts = []
    for gt_line in gt_xyz.readlines():
        gt_l = gt_line.split(',')
        gc_pts.append((float(gt_l[0]), float(gt_l[1].strip('\n'))))
    return np.array(gc_pts)


def load_profile_points(filepath):
    global axle_id

    pts = []
    for a_file in os.listdir(filepath):
        if not a_file.endswith('.json') and a_file.split('_')[0] != str(axle_id):
            continue
        laser_id = a_file.split('_')[2].split(".")[0]
        profile = json.load(open(os.path.join(filepath, a_file), "r"))
        for pt in profile['flange_points']:
            pts.append(pt)

        for pt in profile['tread_points']:
            pts.append(pt)

    # aligned_filename_ply = os.path.join(icp_profile_folder, 'aligned_pts.ply')
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(axy_list)
    # o3d.io.write_point_cloud(aligned_filename_ply, pcd)

    return np.array(pts)


def display_profiles(gc_pts, org_pts, new_pts, winname):
    fig = go.Figure(layout_title_text=winname)
    fig.add_trace(
        go.Scatter(x=gc_pts[:, 0], y=gc_pts[:, 1], mode='markers', marker=dict(symbol='x', size=4),
                   name='gocator flange'))

    fig.add_trace(
        go.Scatter(x=new_pts[:, 0], y=new_pts[:, 1], mode='markers', marker=dict(symbol='square', size=2),
                   name='aligned laser'))

    fig.add_trace(
        go.Scatter(x=org_pts[:, 0], y=org_pts[:, 1], mode='markers', marker=dict(symbol='square', size=2),
                   name='org laser'))
    fig.update_yaxes(scaleanchor='x', scaleratio=1)
    fig.show()


axles = [6]
for axle_id in axles:
    main_folder = r'D:\Project_Data\Field_data\04022022'
    # main_folder = r'D:\Project_Data\Field_data\21-02-2022_13-53-04'
    gocator_tread_filename = main_folder + "\\gocator2d\\" + str(axle_id) + "th_2d_wrt_tread_biased.xyz"
    gocator_flange_filename = main_folder + "\\gocator2d\\" + str(axle_id) + "th_2d_wrt_flange_biased.xyz"
    org_folder = main_folder + "\\Data\\2d_points\\org"
    icp_profile_folder = main_folder + "\\Data\\2d_points\\icp"

    gf_pts = load_gocator_points(gocator_flange_filename)
    gt_pts = load_gocator_points(gocator_tread_filename)

    org_pts = load_profile_points(org_folder)
    new_pts = load_profile_points(icp_profile_folder)

    _, aligned_gf = icp(new_pts, gf_pts, distance_threshold=0.5, verbose=True)
    _, aligned_gt = icp(new_pts, gt_pts, distance_threshold=0.5, verbose=True)

    # compare_profiles(ox_list, ax_list)
    # compare_profiles(oy_list, ay_list)

    # display_profiles_overlapping('icp vs org gocator', new_pts, gt_pts, 'icp', 'org gocator tread')
    # display_profiles_overlapping('icp vs aligned gocator', new_pts, aligned_gt, 'icp', 'aligned gocator tread')
    #
    # display_profiles_overlapping('icp vs org gocator', new_pts, gf_pts, 'icp', 'org gocator flange')
    # display_profiles_overlapping('icp vs aligned gocator', new_pts, aligned_gf, 'icp', 'aligned gocator flange')
    #
    display_profiles(gf_pts, org_pts, new_pts, 'Axle ' + str(axle_id) + ' org gocator w.r.t Flange')
    display_profiles(gt_pts, org_pts, new_pts, 'Axle ' + str(axle_id) + ' org gocator w.r.t Tread')

    display_profiles(aligned_gf, org_pts, new_pts, 'Axle ' + str(axle_id) + ' aligned gocator w.r.t Flange')
    display_profiles(aligned_gt, org_pts, new_pts, 'Axle ' + str(axle_id) + ' aligned gocator w.r.t Tread')
    #
    # write_2d_points_to_file_json(gf_pts, 'flange', main_folder + "\\gocator2d\\", axle_id)
    # write_2d_points_to_file_json(gt_pts, 'tread', main_folder + "\\gocator2d\\", axle_id)
