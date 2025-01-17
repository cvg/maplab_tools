#! /usr/bin/env python2
import numpy as np
import pandas as pd
from os.path import exists


from evo.core.trajectory import PoseTrajectory3D
from evo.core import trajectory
from evo.core import metrics
from evo.core import sync


class PoseTrajectoryEvaluation(object):
    def __init__(self, est_traj_file=None, gt_traj_file=None):
        self.est_df = None
        self.gt_df = None
        if est_traj_file is not None and gt_traj_file is not None:
            self.parse_exported_trajectory(est_traj_file, gt_traj_file)
        elif est_traj_file is not None:
            self.est_df = self.parse_trajectory_csv(est_traj_file)

    def parse_exported_trajectory(self, est_traj_file, gt_traj_file):
        self.est_df = self.parse_trajectory_csv(est_traj_file)
        gt_df = self.parse_trajectory_csv(gt_traj_file)
        self.gt_df = self.synchronize_missions(self.est_df, gt_df)

    def parse_trajectory_csv(self, file):
        header_names = ['timestamp [ns]', 'vertex-id', 'mission-id', 'p_G_Ix [m]', 'p_G_Iy [m]', 'p_G_Iz [m]', 'q_G_Iw', 'q_G_Ix', 'q_G_Iy', 'q_G_Iz', 'p_M_Ix [m]', 'p_M_Iy [m]', 'p_M_Iz [m]',
                        'q_M_Iw', 'q_M_Ix', 'q_M_Iy', 'q_M_Iz', 'v_Mx [m/s]', 'v_My [m/s]', 'v_Mz [m/s]', 'bgx [rad/s]', 'bgy [rad/s]', 'bgz [rad/s]', 'bax [m/s^2]', 'bay [m/s^2]', 'baz [m/s^2]']
        return pd.read_csv(file, names=header_names, delimiter=',', comment='#', header=None)

    def parse_trajectory_txt(self, file, delimiter=' '):
        header_names = ['timestamp [ns]', 'p_G_Ix [m]', 'p_G_Iy [m]',
                        'p_G_Iz [m]', 'q_G_Ix', 'q_G_Iy', 'q_G_Iz', 'q_G_Iw']
        return pd.read_csv(file, names=header_names, delimiter=delimiter, comment='#', header=None)

    def synchronize_missions(self, est_df, gt_df):
        missions = np.array(pd.unique(est_df['mission-id']))
        n_mission = missions.shape[0]
        synced_gt_df = gt_df[gt_df['mission-id'].isin(missions)]
        print('[PoseTrajectoryEvaluation] Synced {n_missions_gt} mission with the GT.'.format(
            n_missions_gt=len(pd.unique(synced_gt_df["mission-id"]))))
        return synced_gt_df

    def get_mission_from_csv(self, file, mission_id):
        df = self.parse_trajectory_csv(file)
        return df[df['mission-id'].str.contains(mission_id)]

    def compute_ape(self):
        est_traj, gt_traj = self.compute_synchronized_trajectories_df()
        return self.compute_trans_ape_rmse(est_traj, gt_traj)

    def compute_synchronized_trajectories_df(self):
        est_traj = self.convert_df_to_traj(self.est_df)
        gt_traj = self.convert_df_to_traj(self.gt_df)
        est_traj, gt_traj = self.synchronize_timestamps(est_traj, gt_traj)
        return est_traj, gt_traj

    def compute_synchronized_trajectories_with_evo2(self, gt_traj):
        est_traj = self.convert_df_to_traj(self.est_df)
        est_evo_traj = self.convert_to_evo_traj(est_traj)
        gt_evo_traj = self.convert_to_evo_traj(gt_traj)
        gt_evo_traj, est_evo_traj = sync.associate_trajectories(
            gt_evo_traj, est_evo_traj, 0.1)

        est_evo_traj = trajectory.align_trajectory(
            est_evo_traj, gt_evo_traj, correct_scale=False, correct_only_scale=False)
        return est_evo_traj, gt_evo_traj

    def compute_synchronized_trajectories_with_evo3(self, gt_traj):
        est_traj = self.convert_df_to_traj(self.est_df)
        est_evo_traj = self.convert_to_evo_traj(est_traj)
        gt_evo_traj = self.convert_to_evo_traj(gt_traj)
        gt_evo_traj, est_evo_traj = sync.associate_trajectories(
            gt_evo_traj, est_evo_traj, 0.1)

        est_evo_traj.align(gt_evo_traj, correct_scale=False,
                           correct_only_scale=False, n=200)
        return est_evo_traj, gt_evo_traj

    def synchronize_timestamps(self, est_traj, gt_traj):
        est_size = est_traj.shape[0]
        gt_size = gt_traj.shape[0]
        min_size = min(est_size, gt_size)
        if min_size > gt_size:
            print(
                '[PoseTrajectoryEvaluation] Estimated size is greater than the GT size.')
            min_size = gt_size
        est_mask = np.zeros((est_size)).astype(np.bool)
        gt_mask = np.zeros((gt_size)).astype(np.bool)

        ts_est = est_traj[:, 0]
        for i in range(min_size):
            cur_ts = gt_traj[i, 0]
            ts_diff = np.absolute(ts_est - cur_ts)
            # TODO(lbern): check for max time diff
            min_ts_diff = np.amin(ts_diff)

            cur_min_idx = np.where(ts_diff == min_ts_diff)[0]
            est_mask[cur_min_idx[0]] = True
            gt_mask[i] = True

        return est_traj[est_mask, :], gt_traj[gt_mask, :]

    def convert_df_to_traj(self, df):
        ts = df['timestamp [ns]'].to_numpy()
        xyz = df[['p_G_Ix [m]', 'p_G_Iy [m]', 'p_G_Iz [m]']].to_numpy()
        wxyz = df[['q_G_Iw', 'q_G_Ix', 'q_G_Iy', 'q_G_Iz']].to_numpy()
        return np.column_stack((ts, xyz, wxyz))

    def ts_ns_to_seconds(self, ts_ns):
        k_ns_per_s = 1e9
        return ts_ns / k_ns_per_s

    def convert_to_evo_traj(self, trajectory):
        ts = self.ts_ns_to_seconds(trajectory[:, 0])
        xyz = trajectory[:, 1:4]
        wxyz = trajectory[:, 4:8]
        return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=wxyz, timestamps=ts)

    def compute_trans_ape_rmse(self, est_traj, gt_traj):
        est_xyz = est_traj[:, 1:4]
        gt_xyz = gt_traj[:, 1:4]

        err = np.linalg.norm(est_xyz - gt_xyz, axis=1)
        return np.sqrt(np.mean(err**2))

    def compute_evo_trans_ape_rmse(self, est_evo_traj, gt_evo_traj):
        data = (gt_evo_traj, est_evo_traj)

        pose_relation = metrics.PoseRelation.translation_part
        ape_metric_trans = metrics.APE(pose_relation)
        ape_metric_trans.process_data(data)
        return ape_metric_trans.get_statistic(metrics.StatisticsType.rmse)


if __name__ == '__main__':
    # merged_map_path = '/tmp/maplab_server/merged_map/'
    # gt_path = '/home/berlukas/Documents/results/hagerbach_july/mission_05_opt/'
    #
    # pose_filename = 'vertex_poses_velocities_biases.csv'
    # est_traj_file = merged_map_path + pose_filename
    # gt_traj_file = gt_path + pose_filename
    #
    # eval = PoseTrajectoryEvaluation(est_traj_file, gt_traj_file)
    # est_traj, gt_traj = eval.compute_synchronized_trajectories_df()
    # print('we have an rmse of {rmse}'.format(rmse=eval.compute_ape()))
    pose_filename = 'vertex_poses_velocities_biases.csv'
    server_path = '/tmp/maplab_server/merged_map/'

    est_traj_file = server_path + pose_filename
    gt_traj_file = '/mnt/data/datasets/fgsp/gt/hagerbach_anymal_2/gt.npy'
    if exists(est_traj_file) and exists(gt_traj_file):
        eval = PoseTrajectoryEvaluation(est_traj_file)
        gt_traj = np.load(gt_traj_file)

        est_traj, gt_traj = eval.compute_synchronized_trajectories_with_evo2(
            gt_traj)
        error = eval.compute_evo_trans_ape_rmse(est_traj, gt_traj)
        print('we have an rmse of {rmse}'.format(rmse=error))
    else:
        print('Input data does not exist.')
