import math

import numpy as np
import plotly.graph_objects as go

from load_data import read_axle_data


def segment_display_tread_profile(tmaster_dict):
    fig = go.Figure()
    for k, tprofile in tmaster_dict.items():
        # if k is not 'left_down':
        #     continue
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
            fig.add_trace(
                go.Scatter(x=[next_pt[0]], y=[next_pt[1]], mode='markers', marker=dict(symbol='x'),
                           name='pt ' + str(cnt)))

        cnt += 50
        fig.add_trace(
            go.Scatter(x=[next_pt[0]], y=[next_pt[1]], mode='markers', marker=dict(symbol='x'),
                       name='pt ' + str(cnt)))

        tread_wall_ids = min_pt_ids[0: cnt]
        tread_surf_ids = min_pt_ids[cnt:]
        tread_wall = np.array([tprofile[i] for i in tread_wall_ids])
        tread_surf = np.array([tprofile[i] for i in tread_surf_ids])

        fig.add_trace(
            go.Scatter(x=tread_wall[:, 0], y=tread_wall[:, 1], mode='markers', marker=dict(symbol='circle', size=2),
                       name='wall ' + str(k)))
        fig.add_trace(
            go.Scatter(x=tread_surf[:, 0], y=tread_surf[:, 1], mode='markers', marker=dict(symbol='circle', size=2),
                       name='surf ' + str(k)))

    fig.show()
    return tread_wall, tread_surf


def run_non_linear_optimization(main_folder, axle_id):
    laser_dict = {'left_down': None, 'left_up': None, 'right_down': None, 'right_up': None}

    tmaster_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}
    twall_master_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}
    tsurf_master_dict = {'org_points': laser_dict.copy(), 'aligned_points': [laser_dict.copy()]}

    _, tmaster_dict['org_points'] = read_axle_data(main_folder, axle_id)

    twall_master_dict['org_points'], tsurf_master_dict['org_points'] = segment_display_tread_profile(
        tmaster_dict['org_points'])


if __name__ == '__main__':
    # Loading input and preparing the dictionaries
    main_folder = r"D:\Project_Data\Field_data\09022022\04-02-2022_14-48-02\Data\Data.json"
    run_non_linear_optimization(main_folder, 6)
