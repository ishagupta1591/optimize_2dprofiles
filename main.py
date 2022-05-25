"""
Data can be laoded from data.json or from disk
You can run ICP on
1. complete profile
2. profile divided into tread and flange sections
3. profile divided into flange and tread wall and tread curve sections
"""

import datetime
import json
import math
import os

from draw_graphs import *
from icp import icp
from load_data import laser_dict, get_vector_separate, get_vector, read_axle_data

load_field_data = True
is_faulty = False
write_2dpoints = False
to_display = not write_2dpoints


# to_display = False


def segment_tread_profile(tprofile):
    min_pt_ids = np.lexsort((tprofile[:, 0], tprofile[:, 1]))

    cnt = 50
    angle = 90
    while angle > 70:
        prev_pt = tprofile[min_pt_ids[cnt]]
        cnt += 30
        next_pt = tprofile[min_pt_ids[cnt]]
        diff_x = (next_pt[0] - prev_pt[0])
        if diff_x == 0:
            angle = 90
        else:
            slope = abs((next_pt[1] - prev_pt[1]) / diff_x)
            angle = math.degrees(math.atan(slope))

    cnt += 50
    tread_wall_ids = min_pt_ids[0: cnt]
    tread_surf_ids = min_pt_ids[cnt:]
    tread_wall = np.array([tprofile[i] for i in tread_wall_ids])
    tread_surf = np.array([tprofile[i] for i in tread_surf_ids])
    return tread_wall, tread_surf


def compute_icp(ref_id, to_be_aligned_id: list, profile_dict: dict, verbose_list: list = None):
    if verbose_list is None:
        verbose_list = [True] * range(to_be_aligned_id)

    ref_profile_mean = np.max(profile_dict[ref_id][:, 0])

    aligned_dict = profile_dict.copy()
    for i, v in zip(to_be_aligned_id, verbose_list):
        print(f"\nReference profile {ref_id} and Target profile {i}")

        target_profile_mean = np.max(profile_dict[i][:, 0])
        dist_thresh = abs(target_profile_mean - ref_profile_mean)
        if dist_thresh == 0:
            dist_thresh = 0.03
        dist_thresh = round(dist_thresh, -int(math.floor(math.log10(abs(dist_thresh)))))

        if v:
            print(f"Ref mean: {ref_profile_mean}\nTarget mean: {target_profile_mean}\nDist Thread: {dist_thresh}")

        _, aligned_dict[i] = icp(profile_dict[ref_id][:, :2], profile_dict[i][:, :2], verbose=v,
                                 distance_threshold=dist_thresh)

    return aligned_dict


def write_2d_points_to_file(fprofiles, tprofiles, save_ply=False, save_dir=None, axle_id=None):
    # if save_dir is None:
    #     save_dir = '2d_points/axle' + str(axle_id)

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    for i in fprofiles:
        profile_dict = {'flange_points': [list(x) for x in fprofiles[i]],
                        'tread_points': [list(x) for x in tprofiles[i]]}
        file_name = '\\laser_profile_' + str(i) + '.json'
        f = open(save_dir + file_name, 'w+')
        f.writelines(json.dumps(profile_dict))

        if save_ply:
            laser_xyz = get_vector(p.flange_profile_2d, p.tread_profile_2d)
            laser_pcd = o3d.geometry.PointCloud()
            laser_pcd.points = o3d.utility.Vector3dVector(laser_xyz)
            o3d.io.write_point_cloud(save_dir + '\\laser_profile_' + str(i) + '.ply', laser_pcd)


def prepare_master_dict(fdict: dict, tdict: dict):
    mdict = {}
    for k in fdict:
        if k in ['left_down', 'left_up'] and is_faulty:
            mdict.update({k: tdict[k]})
        else:
            mdict.update({k: np.concatenate((fdict[k], tdict[k]), axis=0)})
    return mdict


def compare_profiles(org_profile, aligned_profiles):
    for i, j in org_profile.items():
        comp = j == aligned_profiles[i]
        eq_comp = comp.all()
        print(eq_comp)


def run_non_linear_optimization(main_folder, axle_id):
    divide_profile = True  # Divide profile into tread and flange
    divide_tread_profile = True  # Divide tread profile into tread wall and tread surface

    fmaster_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}
    tmaster_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}
    twall_master_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}
    tsurf_master_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}

    master_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}

    if load_field_data:
        fmaster_dict['org_points'], tmaster_dict['org_points'] = read_axle_data(main_folder, axle_id)

        if is_faulty:
            fmaster_dict['org_points']['left_down'] = np.zeros(
                (fmaster_dict['org_points']['left_down'].shape[0], fmaster_dict['org_points']['left_down'].shape[1]))
            fmaster_dict['org_points']['left_up'] = np.zeros(
                (fmaster_dict['org_points']['left_up'].shape[0], fmaster_dict['org_points']['left_up'].shape[1]))

        divide_profile = True
        divide_tread_profile = True

        if divide_tread_profile:
            for k, v in tmaster_dict['org_points'].items():
                twall_master_dict['org_points'][k], tsurf_master_dict['org_points'][k] = segment_tread_profile(v)

        tmaster_dict['org_points'] = prepare_master_dict(twall_master_dict['org_points'],
                                                         tsurf_master_dict['org_points'])
        master_dict['org_points'] = prepare_master_dict(fmaster_dict['org_points'], tmaster_dict['org_points'])

    else:
        if divide_profile:
            fmaster_dict['org_points']['left_down'], tmaster_dict['org_points']['left_down'] = get_vector_separate(
                main_folder + "\\laser_profile_1.json")
            fmaster_dict['org_points']['left_up'], tmaster_dict['org_points']['left_up'] = get_vector_separate(
                main_folder + "\\laser_profile_2.json")
            fmaster_dict['org_points']['right_down'], tmaster_dict['org_points']['right_down'] = get_vector_separate(
                main_folder + "\\laser_profile_3.json")
            fmaster_dict['org_points']['right_up'], tmaster_dict['org_points']['right_up'] = get_vector_separate(
                main_folder + "\\laser_profile_4.json")

            master_dict['org_points'] = prepare_master_dict(fmaster_dict['org_points'], tmaster_dict['org_points'])

            if divide_tread_profile:
                for k, v in tmaster_dict['org_points'].items():
                    twall_master_dict['org_points'][k], tsurf_master_dict['org_points'][k] = segment_tread_profile(v)
        else:
            master_dict['org_points']['left_down'] = get_vector(main_folder + "\\laser_profile_1.json")
            master_dict['org_points']['left_up'] = get_vector(main_folder + "\\laser_profile_2.json")
            master_dict['org_points']['right_down'] = get_vector(main_folder + "\\laser_profile_3.json")
            master_dict['org_points']['right_up'] = get_vector(main_folder + "\\laser_profile_4.json")

    if write_2dpoints:
        write_2d_points_to_file(fmaster_dict['org_points'], tmaster_dict['org_points'], axle_id=axle_id,
                                save_dir="2d_points/round1/org_axle" + str(axle_id))

    # ***********************************************************************************************************
    start_time = datetime.datetime.now()
    # run icp
    if divide_profile:
        print("\nRunning ICP for flange points")
        print("===================================================================================")

        if is_faulty:
            fmaster_dict['aligned_points'][0] = compute_icp('right_down', ['right_up'], fmaster_dict['org_points'], [False])
        else:
            fmaster_dict['aligned_points'][0] = compute_icp('left_down', ['left_up', 'right_down', 'right_up'], fmaster_dict['org_points'],
                                                            [False, False, False])

        if divide_tread_profile:
            print("\nRunning ICP for tread wall points")
            print("===================================================================================")
            twall_master_dict['aligned_points'][0] = compute_icp('left_down', ['left_up', 'right_down', 'right_up'], twall_master_dict['org_points'],
                                                                 [False, False, False])

            print("\nRunning ICP for tread surface points")
            print("===================================================================================")
            tsurf_master_dict['aligned_points'][0] = compute_icp('left_down', ['left_up', 'right_down', 'right_up'], tsurf_master_dict['org_points'],
                                                                 [False, False, False])

            tmaster_dict['aligned_points'][0] = prepare_master_dict(twall_master_dict['aligned_points'][0],
                                                                    tsurf_master_dict['aligned_points'][0])
        else:
            print("\nRunning ICP for tread points")
            print("===================================================================================")
            tmaster_dict['aligned_points'][0] = compute_icp('left_down', ['left_up', 'right_down', 'right_up'], tmaster_dict['org_points'],
                                                            [False, False, False])

        master_dict['aligned_points'][0] = prepare_master_dict(fmaster_dict['aligned_points'][0],
                                                               tmaster_dict['aligned_points'][0])

        # ***********************************************************************************************************

        if to_display:
            # display_profiles_sep_stacked('axle ' + str(axle_id), fmaster_dict['org_points'], tmaster_dict['org_points'],
            #                              fmaster_dict['aligned_points'][0], tmaster_dict['aligned_points'][0])
            display_profiles_sep('Axle ' + str(axle_id) + ' original profiles', fmaster_dict['org_points'],
                                 tmaster_dict['org_points'], filename='images/org_axle' + str(axle_id) + '.png')
            display_profiles_sep('Axle ' + str(axle_id) + ' icp round 1', fmaster_dict['aligned_points'][0],
                                 tmaster_dict['aligned_points'][0], filename='images/icp1_axle' + str(axle_id) + '.png')
            # display_profiles_sep('Axle ' + str(axle_id) + ' icp round 1 tread profiles',
            #                      twall_master_dict['aligned_points'][0], tsurf_master_dict['aligned_points'][0])
            # display_profiles('only wall profiles', twall_master_dict['aligned_points'][0])

            # compare_profiles(fmaster_dict['org_points'], fmaster_dict['aligned_points'][0])
            # compare_profiles(tmaster_dict['org_points'], tmaster_dict['aligned_points'][0])
            # compare_profiles(master_dict['org_points'], master_dict['aligned_points'][0])
            display_profiles_overlapping('overlapping', master_dict['org_points'], master_dict['aligned_points'][0])


    else:
        master_dict['aligned_points'][0] = compute_icp('left_down', ['left_up', 'right_down', 'right_up'], master_dict['org_points'],
                                                       [False, False, False])

        display_profiles('orginal profiles', master_dict['org_points'])
        display_profiles('icp round 1', master_dict['aligned_points'][0])

    end_time = datetime.datetime.now()
    print("time diff - ", end_time - start_time)

    if write_2dpoints:
        write_2d_points_to_file(fmaster_dict['aligned_points'][0], tmaster_dict['aligned_points'][0], axle_id=axle_id,
                                save_dir="2d_points/round1/aligned_axle" + str(axle_id))


if __name__ == '__main__':
    # Loading input and preparing the dictionaries
    # if is_faulty:
    main_folder = r"D:\Project_Data\Field_data\21-02-2022_13-53-04\Data\Data_org.json"
    # main_folder = r"D:\Project_Data\Field_data\04022022\04-02-2022_14-48-02\Data\Data.json"
    # else:
    #     main_folder = r"D:\Project_Data\Field_data\17-01-2022_05-34-57\Data\Data.json"

    to_continue = True
    while to_continue:
        axle_id = int(input("\nEnter axle id: "))
        run_non_linear_optimization(main_folder, axle_id)
        to_continue = input("Do you want to try for another axel (y/n) :").lower() == 'y'
