import math

from draw_graphs import *
from icp import icp
from load_data import laser_dict_spc, read_axle_data, AxleData


def get_end_pt(profile_pts, range, dir):
    """

    :param profile_pts:
    :param range:
    :param dir: if the end point is in the +ve direction on x-axis
    :return:
    """
    delta = 0.0
    while (1):
        if dir == '+':
            end_pts = [x for x in profile_pts if range[0] - delta < x[0] < range[1] + delta]
        else:
            end_pts = [x for x in profile_pts if range[0] + delta > x[0] > range[1] - delta]
        delta += 0.1
        if len(end_pts) > 0:
            break
    end_pt_id = np.where(profile_pts == end_pts[0])
    return end_pt_id


def extract_curve_segments(fprofile, tprofile, to_display=False):
    sorted_flange_pts = np.array(sorted(fprofile, key=lambda x: x[0]))
    sorted_tread_pts = np.array(sorted(tprofile, key=lambda x: x[0]))

    tread_min_x = sorted_tread_pts[0]
    flange_min_x = sorted_flange_pts[-1]

    tread_range = [math.floor(tread_min_x[0] * 10) / 10, math.ceil(tread_min_x[0] * 10) / 10]
    flange_range = [math.floor(flange_min_x[0] * 10) / 10, math.ceil(flange_min_x[0] * 10) / 10]

    if tread_min_x[0] < flange_min_x[0]:
        tread_end_pt_id = get_end_pt(sorted_tread_pts, flange_range, "+")
        flange_end_pt_id = get_end_pt(sorted_flange_pts, tread_range, "-")

        flange_curve = sorted_flange_pts[flange_end_pt_id[0][0]:]
        flange_rest = sorted_flange_pts[0:flange_end_pt_id[0][0]]
        tread_curve = sorted_tread_pts[0:tread_end_pt_id[0][0] + 1]
        tread_rest = sorted_tread_pts[tread_end_pt_id[0][0] + 1:]

    else:
        tread_end_pt_id = get_end_pt(sorted_tread_pts, flange_range, "-")
        flange_end_pt_id = get_end_pt(sorted_flange_pts, tread_range, "+")

        flange_curve = sorted_flange_pts[0:flange_end_pt_id[0][0] + 1]
        flange_rest = sorted_flange_pts[flange_end_pt_id[0][0] + 1:]
        tread_curve = sorted_tread_pts[tread_end_pt_id[0][0]:]
        tread_rest = sorted_tread_pts[0:tread_end_pt_id[0][0]]

    if to_display:
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=fprofile[:, 0], y=fprofile[:, 1], mode='markers', marker=dict(symbol='square'),
                                 name='flange Laser'))
        fig.add_trace(go.Scatter(x=tprofile[:, 0], y=tprofile[:, 1], mode='markers', marker=dict(symbol='square'),
                                 name='tread Laser'))
        fig.add_trace(
            go.Scatter(x=[tread_min_x[0]], y=[tread_min_x[1]], mode='markers', marker=dict(symbol='x', size=10),
                       name='tread pt'))
        fig.add_trace(
            go.Scatter(x=[flange_min_x[0]], y=[flange_min_x[1]], mode='markers', marker=dict(symbol='x', size=10),
                       name='flange pt'))
        fig.add_trace(
            go.Scatter(x=[sorted_tread_pts[tread_end_pt_id[0][0]][0]], y=[sorted_tread_pts[tread_end_pt_id[0][0]][1]],
                       mode='markers', marker=dict(symbol='x', size=10), name='tread end pt'))
        fig.add_trace(
            go.Scatter(x=[sorted_tread_pts[tread_end_pt_id[0][0]][0]], y=[sorted_tread_pts[tread_end_pt_id[0][0]][1]],
                       mode='markers', marker=dict(symbol='x', size=10), name='flange end pt'))

        fig.add_trace(
            go.Scatter(x=flange_curve[:, 0], y=flange_curve[:, 1], mode='markers', marker=dict(symbol='diamond'),
                       name='flange curve'))
        fig.add_trace(
            go.Scatter(x=tread_curve[:, 0], y=tread_curve[:, 1], mode='markers', marker=dict(symbol='diamond'),
                       name='tread curve'))

        fig.show()
    return flange_curve, flange_rest, tread_curve, tread_rest


def prepare_master_dict(set1, set2, is_dict=True):
    if is_dict:
        mdict = {}
        for k in set1:
            mdict.update({k: np.concatenate((set1[k], set2[k]), axis=0)})
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
    fmaster_dict = {'org_points': laser_dict_spc.copy(), 'aligned_points': []}
    tmaster_dict = {'org_points': laser_dict_spc.copy(), 'aligned_points': []}
    master_dict = {'org_points': [], 'aligned_points': []}

    axleData = AxleData()
    axleData = read_axle_data(main_folder, axle_id)
    fmaster_dict['org_points']['right_up'] = axleData.flange_profile2d['right_up']
    tmaster_dict['org_points']['right_up'] = axleData.tread_profile2d['right_up']

    master_dict['org_points'] = prepare_master_dict(fmaster_dict['org_points'], tmaster_dict['org_points'])

    display_profiles_sep('orginal', fmaster_dict['org_points'], tmaster_dict['org_points'])
    # -------------------GET CURVE--------------------------------------------------------------------------------------
    flange_curve, flange_rest, tread_curve, tread_rest = extract_curve_segments(
        fmaster_dict['org_points']['right_up'],
        tmaster_dict['org_points']['right_up'], False)
    display_profiles_sep('orginial', {'curve': flange_curve, 'rest': flange_rest},
                         {'curve': tread_curve, 'rest': tread_rest})

    # --------------------RUN ICP---------------------------------------------------------------------------------------
    # --------------------------------------Complete Profile------------------
    # --------------distance threshold calculated based on the complete overlap logic
    # tmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    # master_dict['aligned_points'].append(laser_dict_spc.copy())
    #
    # tmaster_dict['aligned_points'][-1]['right_up'] = compute_icp(fmaster_dict['org_points']['right_up'],
    #                                                       tmaster_dict['org_points']['right_up'])
    #
    # display_profiles_sep('aligned complete dist thres calc', fmaster_dict['org_points'],
    #                      tmaster_dict['aligned_points'][-1])

    # ---------distance threshold specified
    tmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    master_dict['aligned_points'].append(laser_dict_spc.copy())

    _, tmaster_dict['aligned_points'][-1]['right_up'] = icp(fmaster_dict['org_points']['right_up'][:, :2],
                                                            tmaster_dict['org_points']['right_up'][:, :2],
                                                            distance_threshold=0.7)

    display_profiles_sep('aligned complete dist thres given', fmaster_dict['org_points'],
                         tmaster_dict['aligned_points'][-1])

    # --------------------------------------Curves-----------
    # # rotation and translation
    # trans_curve_both, tread_curve_new_both = icp(flange_curve, tread_curve, distance_threshold=0.7)
    # tread_rest_new_both = tread_rest.copy()
    # for t in trans_curve_both:
    #     tread_rest_new_both = np.dot(tread_rest_new_both, t[:, :2].T)
    #     tread_rest_new_both[:, 0] += t[:, -1][0]
    #     tread_rest_new_both[:, 1] += t[:, -1][1]
    #
    # tmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    # master_dict['aligned_points'].append(laser_dict_spc.copy())
    #
    # tmaster_dict['aligned_points'][-1]['right_up'] = prepare_master_dict(tread_curve_new_both, tread_rest_new_both,
    #                                                                        False)
    # master_dict['aligned_points'][-1] = prepare_master_dict(fmaster_dict['org_points'],
    #                                                         tmaster_dict['aligned_points'][-1])
    #
    # display_profiles_sep('aligned curve both rotation and translation', {'curve': flange_curve, 'rest': flange_rest},
    #                      {'curve': tread_curve_new_both, 'rest': tread_rest_new_both})
    #
    # # only translation
    # trans_curve, tread_curve_new = icp_trans(flange_curve, tread_curve, distance_threshold=0.7, point_pairs_threshold=3)
    #
    # tread_rest_new = tread_rest.copy()
    # for t in trans_curve:
    #     tread_rest_new[:, 0] += t[:, -1][0]
    #     tread_rest_new[:, 1] += t[:, -1][1]
    #
    # tmaster_dict['aligned_points'].append(laser_dict_spc.copy())
    # master_dict['aligned_points'].append(laser_dict_spc.copy())
    #
    # tmaster_dict['aligned_points'][-1]['right_up'] = prepare_master_dict(tread_curve_new, tread_rest_new, False)
    # master_dict['aligned_points'][-1] = prepare_master_dict(fmaster_dict['org_points'],
    #                                                         tmaster_dict['aligned_points'][1])
    #
    # display_profiles_sep('aligned curve only translation', {'curve': flange_curve, 'rest': flange_rest},
    #                      {'curve': tread_curve_new, 'rest': tread_rest_new})
    # ------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
    main_folder = r"D:\Project_Data\Field_data\21-02-2022_13-53-04\Data\Data_org.json"
    run_non_linear_optimization(main_folder, 0)
