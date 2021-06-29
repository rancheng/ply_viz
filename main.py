import os
import numpy as np
from glob import glob
import open3d as o3d
import argparse
import time
import pyquaternion


# https://github.com/intel-isl/Open3D/issues/2#issuecomment-610683341
def text_3d(text, pos, direction=None, degree=0.0, font="OpenSans-Regular.ttf", font_size=16):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param pos: 3D xyz position of the text upper left corner
    :param direction: 3D normalized direction of where the text faces
    :param degree: in plane rotation of text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """
    if direction is None:
        direction = (0., 0., 1.)

    from PIL import Image, ImageFont, ImageDraw
    from pyquaternion import Quaternion

    font_obj = ImageFont.truetype(font, font_size)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0)

    raxis = np.cross([0.0, 0.0, 1.0], direction)
    if np.linalg.norm(raxis) < 1e-6:
        raxis = (0.0, 0.0, 1.0)
    trans = (Quaternion(axis=raxis, radians=np.arccos(direction[2])) *
             Quaternion(axis=direction, degrees=degree)).transformation_matrix
    trans[0:3, 3] = np.asarray(pos)
    pcd.transform(trans)
    return pcd


def visualize_pcd_auto_play(cfg):
    WINDOW_WIDTH = cfg.window_width  # change this if needed
    WINDOW_HEIGHT = cfg.window_height  # change this if needed
    file_type = cfg.file_type
    ply_base_dir = cfg.ply_folder
    ply_files = [pf for pf in os.listdir(ply_base_dir) if pf.endswith(file_type)]
    if not len(ply_files) > 0:
        print("empty directory, exiting...")
        return
    # first pcd file
    source_pcd = o3d.io.read_point_cloud(os.path.join(ply_base_dir, ply_files[0]))
    min_pos = np.min(source_pcd.points, axis=0)
    max_pos = np.max(source_pcd.points, axis=0)
    shift_diff_pos = (max_pos - min_pos) / 10
    text_pos = min_pos - shift_diff_pos
    text_pcd = text_3d(text="idx: {} | file: {}".format(0, ply_files[0]), pos=text_pos)
    tmp_pcd_text_points = np.vstack([source_pcd.points, text_pcd.points])
    source_pcd.points = o3d.utility.Vector3dVector(tmp_pcd_text_points)
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT)
    vis.add_geometry(source_pcd)
    ctr = vis.get_view_control()
    parameters = o3d.io.read_pinhole_camera_parameters(cfg.camera_view)
    ctr.convert_from_pinhole_camera_parameters(parameters)
    for idx in range(1, len(ply_files)):
        tmp_pcd = o3d.io.read_point_cloud(os.path.join(ply_base_dir, ply_files[idx]))
        min_pos = np.min(tmp_pcd.points, axis=0)
        max_pos = np.max(tmp_pcd.points, axis=0)
        shift_diff_pos = (max_pos - min_pos) / 10
        text_pos = min_pos - shift_diff_pos
        text_pcd = text_3d(text="idx: {} | file: {}".format(0, ply_files[0]), pos=text_pos)
        tmp_pcd_text_points = np.vstack([tmp_pcd.points, text_pcd.points])
        source_pcd.points = o3d.utility.Vector3dVector(tmp_pcd_text_points)
        vis.update_geometry(source_pcd)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(cfg.pause_time)


def visualize_pcd_with_key_bind(cfg):
    WINDOW_WIDTH = cfg.window_width  # change this if needed
    WINDOW_HEIGHT = cfg.window_height  # change this if needed
    file_type = cfg.file_type
    ply_base_dir = cfg.ply_folder
    ply_files = [pf for pf in os.listdir(ply_base_dir) if pf.endswith(file_type)]
    if not len(ply_files) > 0:
        print("empty directory, exiting...")
        return
    source_pcd = o3d.io.read_point_cloud(os.path.join(ply_base_dir, ply_files[0]))
    min_pos = np.min(source_pcd.points, axis=0)
    max_pos = np.max(source_pcd.points, axis=0)
    shift_diff_pos = (max_pos - min_pos) / 10
    text_pos = min_pos - shift_diff_pos
    text_pcd = text_3d(text="idx: {} | file: {}".format(0, ply_files[0]), pos=text_pos)
    tmp_pcd_text_points = np.vstack([source_pcd.points, text_pcd.points])
    source_pcd.points = o3d.utility.Vector3dVector(tmp_pcd_text_points)
    num_ply_file = len(ply_files)
    rotating = False
    pointer = 0
    vis = o3d.visualization.VisualizerWithKeyCallback()

    def key_action_callback(vis, action, mods):
        nonlocal rotating
        # print(action)
        # print(o3d.visualization.gui.KeyEvent.type)
        if action == 1:  # key down
            rotating = True
        elif action == 0:  # key up
            rotating = False
        return True

    def animation_callback(vis):
        nonlocal rotating
        nonlocal pointer
        nonlocal num_ply_file
        nonlocal ply_base_dir
        nonlocal ply_files
        if rotating:
            pointer += 1
            cur_ply_f_idx = pointer % num_ply_file
            tmp_pcd = o3d.io.read_point_cloud(os.path.join(ply_base_dir, ply_files[cur_ply_f_idx]))
            min_pos = np.min(tmp_pcd.points, axis=0)
            max_pos = np.max(tmp_pcd.points, axis=0)
            shift_diff_pos = (max_pos - min_pos) / 10
            text_pos = min_pos - shift_diff_pos
            text_pcd = text_3d(text="idx: {} | file: {}".format(cur_ply_f_idx, ply_files[cur_ply_f_idx]), pos=text_pos)
            tmp_pcd_text_points = np.vstack([tmp_pcd.points, text_pcd.points])
            source_pcd.points = o3d.utility.Vector3dVector(tmp_pcd_text_points)
            vis.update_geometry(source_pcd)
            vis.poll_events()
            vis.update_renderer()
            rotating = False

    # key_action_callback will be triggered when there's a keyboard press, release or repeat event
    vis.register_key_action_callback(32, key_action_callback)  # space

    # animation_callback is always repeatedly called by the visualizer
    vis.register_animation_callback(animation_callback)

    vis.create_window(width=WINDOW_WIDTH, height=WINDOW_HEIGHT)
    vis.add_geometry(source_pcd)
    vis.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Ply Visualizer')
    parser.add_argument('--keyboard', type=bool, default=False,
                        help='Use Keyboard interaction or auto-play.')
    parser.add_argument('--ply_folder', type=str, default="./ply",
                        help='the folder of ply files')
    parser.add_argument('--file_type', type=str, default=".ply",
                        help='the folder of ply files')
    parser.add_argument('--window_width', type=int, default="640",
                        help='the width of visualization window')
    parser.add_argument('--pause_time', type=float, default="0.2",
                        help='time to sleep for each frame, only effective in auto-play mode')
    parser.add_argument('--window_height', type=int, default="480",
                        help="the height of visualization window")
    parser.add_argument('--camera_view', type=str, default="ScreenCamera_2021-06-29-14-56-27.json",
                        help='The screen camera pose json file, if you don\'t have one record it with key P when you are in an open3d window')
    args = parser.parse_args()
    if args.keyboard:
        visualize_pcd_with_key_bind(args)
    else:
        visualize_pcd_auto_play(args)
