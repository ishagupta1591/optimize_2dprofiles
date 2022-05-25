import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def display_profiles(window_name, profile_dict: dict):
    """
    Function to display 4 laser profiles
    :param window_name: name of the graph
    :param profile_dict: profile dictionary containing 4 laser profiles
    :return: None
    """
    fig = go.Figure(layout_title_text=window_name)
    for k, v in profile_dict.items():
        fig.add_trace(go.Scatter(x=v[:, 0], y=v[:, 1], mode='markers', marker=dict(symbol='square'), name='Laser ' + k))
    fig.show()


def display_profiles_stacked(window_name, profile_dict_org: dict, profile_dict: dict):
    """
    Function to display two sets of complete profiles stacked vertically
    :param window_name: name of the graph
    :param profile_dict_org: set 1 containing 4 laser profiles
    :param profile_dict: set 2 containing 4 laser profiles
    :return: None
    """
    fig = make_subplots(rows=2, cols=1)
    for k in profile_dict:
        fig.add_trace(go.Scatter(x=profile_dict_org[k][:, 0], y=profile_dict_org[k][:, 1], mode='markers',
                                 marker=dict(symbol='square'), name='Org Laser ' + k), row=1, col=1)
    fig.add_trace(
        go.Scatter(x=profile_dict[k][:, 0], y=profile_dict[k][:, 1], mode='markers', marker=dict(symbol='square'),
                   name='Laser ' + k), row=2, col=1)
    fig.show()


def display_profiles_sep(window_name, flange_dict: dict, tread_dict: dict, filename=None, is_3d=False):
    """
    Function to display one set of laser profiles, flange and tread separately
    :param window_name: name of the graph
    :param flange_dict: dictionary containing 4 laser flange profiles
    :param tread_dict: dictionary containing 4 laser tread profiles
    :param filename: name/path where the figure is to be written on the disk
    :return: None
    """
    fig = go.Figure(layout_title_text=window_name)
    for k in flange_dict:
        if is_3d:
            fig.add_trace(
                go.Scatter3d(x=flange_dict[k][:, 0], y=flange_dict[k][:, 1], z=flange_dict[k][:, 2],
                             marker=dict(symbol='square', size=2), name='flange ' + k))
            fig.add_trace(
                go.Scatter3d(x=tread_dict[k][:, 0], y=tread_dict[k][:, 1], z=tread_dict[k][:, 2],
                             marker=dict(symbol='square', size=2), name='tread ' + k))
        else:
            fig.add_trace(
                go.Scatter(x=flange_dict[k][:, 0], y=flange_dict[k][:, 1], mode='markers', marker=dict(symbol='square'),
                           name='flange ' + k))
            fig.add_trace(
                go.Scatter(x=tread_dict[k][:, 0], y=tread_dict[k][:, 1], mode='markers', marker=dict(symbol='square'),
                           name='tread ' + k))

    # if not is_3d:
    #     fig.update_yaxes(scaleanchor="x", scaleratio=1)

    fig.show()
    if filename is not None:
        fig.write_image(filename)


def display_profiles_sep_stacked(window_name, flange_dict: dict, tread_dict: dict):
    """
    Function to display multiple sets of laser profiles, flange and tread separately, stacked vertically
    :param window_name: name of the graph
    :param flange_dict: dictionary containing flange profiles for each set
    :param tread_dict: dictionary containing tread profiles for each set
    :return: None
    """
    fig = make_subplots(rows=len(flange_dict), cols=1)

    for i in range(len(flange_dict)):
        for k in flange_dict[i]:
            fig.add_trace(go.Scatter(x=flange_dict[i][k][:, 0], y=flange_dict[i][k][:, 1], mode='markers',
                                     marker=dict(symbol='square'), name='Flange ' + k + ' Set ' + str(i + 1)),
                          row=i + 1, col=1)
            fig.add_trace(go.Scatter(x=tread_dict[i][k][:, 0], y=tread_dict[i][k][:, 1], mode='markers',
                                     marker=dict(symbol='square'), name='Tread ' + k + ' Set ' + str(i + 1)),
                          row=i + 1, col=1)
    fig.show()


def display_profiles_overlapping(window_name, set1_dict, set2_dict, set1_name='org', set2_name='aligned'):
    """
    Function to display two sets of laser profiles in the same fig
    :param window_name: name of the graph
    :param set1_dict: set 1 of laser profiles
    :param set2_dict: set 2 of laser profiles
    :return: None
    """

    if isinstance(set1_dict, dict) and isinstance(set2_dict, dict):
        set1_pts = np.concatenate((list(set1_dict.values())), axis=0)
        set2_pts = np.concatenate((list(set2_dict.values())), axis=0)
    else:
        set1_pts = set1_dict.copy()
        set2_pts = set2_dict.copy()

    fig = go.Figure(layout_title_text=window_name)
    fig.add_trace(
        go.Scatter(x=set1_pts[:, 0], y=set1_pts[:, 1], mode='markers', marker=dict(symbol='square', size=2),
                   name=set1_name))
    fig.add_trace(
        go.Scatter(x=set2_pts[:, 0], y=set2_pts[:, 1], mode='markers', marker=dict(symbol='square', size=2),
                   name=set2_name))
    fig.show()


def display_given_profiles(window_name, profiles: list, is_3d=False):
    """
    Function to display the given profiles
    :param window_name: name of the graph
    :param profiles: profile list
    :param is_3d: boolean value, True: 3d graph, False: 2d graph
    :return: None
    """
    fig = go.Figure(layout_title_text=window_name)
    cnt = 1
    for p in profiles:
        if is_3d:
            fig.add_trace(
                go.Scatter3d(x=p[:, 0], y=p[:, 1], z=p[:, 2], mode='markers', marker=dict(symbol='square', size=2),
                             name=str(cnt)))
        else:
            fig.add_trace(go.Scatter(x=p[:, 0], y=p[:, 1], mode='markers', marker=dict(symbol='square', size=2),
                                     name=str(cnt)))
        cnt += 1
    fig.show()


def display_profile_wall_planes(window_name, flange_dict, tread_dict, flange_plane_list, tread_plane_list):
    fig = go.Figure(layout_title_text=window_name)
    for k in flange_dict:
        fig.add_trace(
            go.Scatter3d(x=flange_dict[k][:, 0], y=flange_dict[k][:, 1], z=flange_dict[k][:, 2],
                         marker=dict(symbol='square', size=2), name='flange ' + k))
        fig.add_trace(
            go.Scatter3d(x=tread_dict[k][:, 0], y=tread_dict[k][:, 1], z=tread_dict[k][:, 2],
                         marker=dict(symbol='square', size=2), name='tread ' + k))

    fig.add_trace(
        go.Surface(x=flange_plane_list[:, 0], y=flange_plane_list[:, 1], z=flange_plane_list[:, 2], showscale=False,
                   name='Flange wall'))
    fig.add_trace(
        go.Surface(x=tread_plane_list[:, 0], y=tread_plane_list[:, 1], z=tread_plane_list[:, 2], showscale=False,
                   name='Tread Wall'))

    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    fig.show()
