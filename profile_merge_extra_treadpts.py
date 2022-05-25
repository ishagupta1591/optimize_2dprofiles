import json
import math
import os

from draw_graphs import *
from icp import icp
from load_data import AxleData, read_axle_data, laser_dict_spc, get_vector_separate

from_datajson = False


def write_2d_points_to_file(fprofiles, tprofiles, save_dir, axle_id):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    file_name = save_dir + '\\' + str(axle_id) + '_ideal_flange_profile.txt'
    np.savetxt(file_name, fprofiles)

    file_name = save_dir + '\\' + str(axle_id) + '_ideal_tread_profile.txt'
    np.savetxt(file_name, tprofiles)


def write_2d_points_to_file_json(fprofiles, tprofiles, save_dir=None, axle_id=None):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    for i in fprofiles:
        profile_dict = {'flange_points': [list(x) for x in fprofiles[i][:, :2]],
                        'tread_points': [list(x) for x in tprofiles[i][:, :2]]}
        file_name = '\\' + str(axle_id) + '_laser_profile_' + str(i) + '.json'
        f = open(save_dir + file_name, 'w+')
        f.writelines(json.dumps(profile_dict))


def prepare_master_dict(set1, set2, is_dict=True):
    if is_dict:
        mdict = {}
        for k in set1:
            mdict.update({k: np.concatenate((set1[k][:, :2], set2[k][:, :2]), axis=0)})
        return mdict
    else:
        return np.concatenate((set1, set2), axis=0)


def compute_icp(ref_profile, target_profile):
    ref_profile_mean = np.max(ref_profile[:, 0])
    target_profile_mean = np.max(target_profile[:, 0])

    dist_thresh = abs(target_profile_mean - ref_profile_mean)
    if dist_thresh == 0:
        dist_thresh = 0.03

    dist_thresh = round(dist_thresh, -int(math.floor(math.log10(abs(dist_thresh)))))

    _, aligned_pts = icp(ref_profile[:, :2], target_profile[:, :2], verbose=False, distance_threshold=dist_thresh)
    return aligned_pts


def run_non_linear_optimization(main_folder, axle_id):
    main_folder_2d = os.path.join(main_folder, '2d_points/')
    fmaster_dict = {'org_points_datajson': laser_dict_spc.copy(), 'org_points': laser_dict_spc.copy(),
                    'aligned_points': []}
    tmaster_dict = {'org_points_datajson': laser_dict_spc.copy(), 'org_points': laser_dict_spc.copy(),
                    'aligned_points': []}
    master_dict = {'org_points': [], 'aligned_points': []}

    axleData = AxleData()
    axleData = read_axle_data(os.path.join(main_folder, 'Data.json'), axle_id)

    fmaster_dict['org_points_datajson']['right_up'] = axleData.flange_profile2d['right_up']
    tmaster_dict['org_points_datajson']['right_up'] = axleData.tread_profile2d['right_up']

    if from_datajson:
        fmaster_dict['org_points']['right_up'] = axleData.flange_profile2d['right_up'][:, :2]
        tmaster_dict['org_points']['right_up'] = axleData.tread_profile2d['right_up'][:, :2]
    else:
        file_path = os.path.join(main_folder_2d, 'cal/', str(axle_id) + "_laser_profile_right_up.json")
        fmaster_dict['org_points']['right_up'], tmaster_dict['org_points']['right_up'] = get_vector_separate(file_path)

    master_dict['org_points'] = prepare_master_dict(fmaster_dict['org_points'], tmaster_dict['org_points'])
    # --------------------RUN ICP---------------------------------------------------------------------------------------
    fmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    tmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    master_dict['aligned_points'].append(laser_dict_spc.copy())
    _, tmaster_dict['aligned_points'][-1]['right_up'] = icp(fmaster_dict['org_points']['right_up'][:, :2],
                                                            tmaster_dict['org_points']['right_up'][:, :2],
                                                            distance_threshold=0.7)

    master_dict['aligned_points'][-1] = prepare_master_dict(fmaster_dict['org_points'],
                                                            tmaster_dict['aligned_points'][-1])
    display_profiles_overlapping('before vs after icp', master_dict['org_points'], master_dict['aligned_points'][-1],
                                 'before icp', 'after icp')

    write_2d_points_to_file(fmaster_dict['org_points']['right_up'][:, :2],
                            tmaster_dict['aligned_points'][-1]['right_up'][:, :2],
                            os.path.join(main_folder_2d, 'ideal_profiles'), axle_id)
    write_2d_points_to_file_json(fmaster_dict['org_points'], tmaster_dict['aligned_points'][-1],
                                 os.path.join(main_folder_2d, 'icp'), axle_id)
    write_2d_points_to_file_json(fmaster_dict['org_points_datajson'], tmaster_dict['org_points_datajson'],
                                 os.path.join(main_folder_2d, 'org'), axle_id)


if __name__ == '__main__':
    main_folder = r'D:\Project_Data\Field_data\21-02-2022_13-53-04\Data'
    run_non_linear_optimization(main_folder, 5)
