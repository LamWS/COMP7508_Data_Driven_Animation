from file_io import BVHMotion
from matching_utils import *
from Viewer.controller import SimpleViewer, Controller

import numpy as np
import scipy.signal as signal
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R


feature_mapping = {
    'lFootPos': 3,
    'rFootPos': 3,
    'lFootVel': 3,
    'rFootVel': 3,
    'lFootRot': 3,
    'rFootRot': 3,
    'lHandPos': 3,
    'rHandPos': 3,
    'lHandVel': 3,
    'rHandVel': 3,
    'lHandRot': 3,
    'rHandRot': 3,
    'rHipPos': 3,
    'lHipPos': 3,
    'rHipVel': 3,
    'lHipVel': 3,
    'rHipRot': 3,
    'lHipRot': 3,
    'hipPos': 3,
    'hipRot': 3,
    'hipVel': 3,
    'trajectoryPos2D': 6,
    'trajectoryRot2D': 6,
    'lKneePos': 3,
    
}

class Database():
    def __init__(self, motions, feature_mapping, selected_feature_names, selected_feature_weights) -> None:
        
        self.dt = 1/60
        
        # Feature Definition
        self.feature_dims = [feature_mapping[i] for i in selected_feature_names]
        self.feature_names = selected_feature_names
        self.feature_weights = []
        
        self.feature_start_idxs = [0]
        for idx, dim_num in enumerate(self.feature_dims):
            self.feature_start_idxs.append(self.feature_start_idxs[-1]+dim_num)
            self.feature_weights.extend([selected_feature_weights[idx]]*dim_num)
        
        self.feature_weights = np.array(self.feature_weights)
        
        self.joint_name, self.joint_parent, self.joint_offset = motions[0].joint_name, motions[0].joint_parent, motions[0].joint_offset
        self.joint_name = ['Simulation_Bone'] + self.joint_name
        self.joint_parent = [-1] + [i+1 for i in self.joint_parent]
        self.joint_offset = np.concatenate([[np.zeros((1,3))], self.joint_offset], axis=0)[:, 0]
        self.joint_channel = np.array([3 if 'end' not in name else 0 for name in self.joint_name])
        self.joint_channel[:2] = 6
        self.joint_channel_offset = np.cumsum([0] + list(self.joint_channel))
        
        # Load Motion Data 
        self.motion_data = []
        self.features = []
        self.local_velocities = []
        self.local_a_velocities = []
        
        for bvh_item in motions:
            joint_name = bvh_item.joint_name
            motion_data = bvh_item.motion_data
            true_motion_data = np.zeros((motion_data.shape[0], self.joint_channel_offset[-1]))
            joint_channel = np.array([3 if 'end' not in name else 0 for name in joint_name])
            joint_channel[0] = 6
            joint_channel_offset = np.cumsum([0] + list(joint_channel))
            
            for idx, name in enumerate(joint_name):
                if joint_channel[idx] == 0:
                    continue
                true_idx = self.joint_name.index(name)
                true_motion_data[:, self.joint_channel_offset[true_idx]:self.joint_channel_offset[true_idx+1]] = motion_data[:, joint_channel_offset[idx]:joint_channel_offset[idx+1]]
        
            # Add a simulation bone
            sim_position_joint = self.joint_name.index('lowerback_torso')
            sim_rotation_joint = self.joint_name.index('RootJoint')
            tmp_pos, tmp_rot = forward_kinematics_with_channel(self.joint_parent, self.joint_channel, self.joint_offset, true_motion_data)
            sim_position = tmp_pos[:, sim_position_joint, :]

            sim_position = np.array([1, 0, 1]) * sim_position
            sim_rotation = R.from_quat(tmp_rot[:, sim_rotation_joint, :])
            fwd_direction = np.array([1,0,1]) * sim_rotation.apply(np.array([0, 0, 1]))
            fwd_direction = fwd_direction / np.linalg.norm(fwd_direction, axis=1, keepdims=True)
            
            filtered_sim_position = signal.savgol_filter(sim_position, 31, 3, axis=0, mode='interp')
            filtered_fwd_direction = signal.savgol_filter(fwd_direction, 61, 3, axis=0, mode='interp')
            filtered_fwd_direction = filtered_fwd_direction / np.linalg.norm(filtered_fwd_direction, axis=1, keepdims=True)
            
            angle = np.arctan2(filtered_fwd_direction[:, 0], filtered_fwd_direction[:, 2])
            filtered_sim_rotation = R.from_rotvec(np.array([0,1,0]).reshape(1,3) * angle[:, np.newaxis])
            true_motion_data[:,:3] = filtered_sim_position
            true_motion_data[:,3:6] = filtered_sim_rotation.as_euler('XYZ', degrees=True)
            true_motion_data[:, 6:9] -= filtered_sim_position
            true_motion_data[:, 9:12] = (filtered_sim_rotation.inv() * R.from_euler('XYZ', true_motion_data[:, 9:12], degrees=True)).as_euler('XYZ', degrees=True)
 
            self.motion_data.append(true_motion_data)
        
            # Extract the data terms 
            pos, rot = forward_kinematics_with_channel(self.joint_parent, self.joint_channel, self.joint_offset, true_motion_data)
            rot = align_quat(rot, False)
            vel = np.zeros_like(pos)
            vel[1:] = (pos[1:] - pos[:-1])/self.dt
            vel[0] = vel[-1]
            avel = np.zeros_like(vel)
            avel[1:] = quat_to_avel(rot, self.dt)
            avel[0] = avel[-1]
        
            # Extract the features
            features = self.extract_features(pos[:, 0], rot[:, 0], pos, rot, vel, avel)
            self.features.append(features)
            
            # Store the local variables
            local_rot = R.from_euler('XYZ', true_motion_data.reshape(-1,3), degrees=True).as_quat().reshape(true_motion_data.shape[0], -1, 4)
            local_rot = align_quat(local_rot, False)
            avel = np.zeros_like(true_motion_data)
            avel[1:] = quat_to_avel(local_rot, self.dt).reshape(-1, avel.shape[-1])
            avel[0] = avel[-1]
            self.local_a_velocities.append(avel)

            vel = np.zeros_like(true_motion_data)
            vel[1:] = (true_motion_data[1:] - true_motion_data[:-1])/self.dt
            vel[0] = vel[-1]
            self.local_velocities.append(vel)
            
        self.motion_data = np.concatenate(self.motion_data, axis=0)
        self.features = np.concatenate(self.features, axis=0)
        self.local_a_velocities = np.concatenate(self.local_a_velocities, axis=0)
        self.local_velocities = np.concatenate(self.local_velocities, axis=0)
        
        self.frame_num, self.joint_num = self.motion_data.shape[0], self.local_a_velocities.shape[1] // 3
        
        self.features_mean, self.features_std = np.mean(self.features, axis=0), np.std(self.features, axis=0)
        self.features_std[self.features_std <= 0.1] = 0.1
        
        self.normalized_feature = (self.features - self.features_mean) / self.features_std
        self.query_tree = cKDTree(self.normalized_feature * self.feature_weights)
        
        print('Feature Extraction Done')
        
    def extract_features(self, root_pos, root_rot, pos, rot, vel, avel):
        features = []
        for feature_name in self.feature_names:
            if feature_name == 'lFootPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('lToeJoint')]))
            elif feature_name == 'rFootPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('rToeJoint')]))
            elif feature_name == 'lFootRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('lToeJoint')]))
            elif feature_name == 'rFootRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('rToeJoint')]))
            elif feature_name == 'lFootVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('lToeJoint')]))
            elif feature_name == 'rFootVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('rToeJoint')]))
            elif feature_name == 'lHandPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('lWrist')]))
            elif feature_name == 'rHandPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('rWrist')]))
            elif feature_name == 'lHandRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('lWrist')]))
            elif feature_name == 'rHandRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('rWrist')]))
            elif feature_name == 'lHandVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('lWrist')]))
            elif feature_name == 'rHandVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('rWrist')]))
            elif feature_name == 'rHipPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('lHip')]))
            elif feature_name == 'lHipPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('rHip')]))
            elif feature_name == 'rHipRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('lHip')]))
            elif feature_name == 'lHipRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('rHip')]))
            elif feature_name == 'rHipVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('lHip')]))
            elif feature_name == 'lHipVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('rHip')]))
            elif feature_name == 'hipPos':
                features.append(self.extract_offset(root_pos, root_rot, pos[:, self.joint_name.index('RootJoint')]))
            elif feature_name == 'hipRot':
                features.append(self.extract_rotation(root_rot, rot[:, self.joint_name.index('RootJoint')]))
            elif feature_name == 'hipVel':
                features.append(self.extract_vel(root_rot, vel[:, self.joint_name.index('RootJoint')]))
            elif feature_name == 'trajectoryPos2D':
                features.append(self.extract_future_pos(root_pos, root_rot, pos[:, 0]))
            elif feature_name == 'trajectoryRot2D':
                features.append(self.extract_future_rot(root_rot, rot[:, 0]))
        return np.concatenate(features, axis=-1)

    def calculate_statistics(self):
        self.features_mean = np.mean(self.features, axis=0)
        self.features_std = np.std(self.features, axis=0)
        self.features_std[self.features_std <= 0.1] = 0.1
    
    def normalize_features(self, features):
        normalized_feature = (features - self.features_mean) / self.features_std
        return normalized_feature

    def extract_vel(self, root_rot, bone_vel):
        rot = R.from_quat(root_rot).inv()
        return rot.apply(bone_vel)

    def extract_offset(self, root_pos, root_rot, bone_pos):
        rot = R.from_quat(root_rot).inv()
        return rot.apply(bone_pos - root_pos)

    def extract_rotation(self, root_rot, bone_rot):
        rot = R.from_quat(root_rot).inv()
        return (rot * R.from_quat(bone_rot)).as_euler('XYZ', degrees=True)

    def extract_future_pos(self, root_pos, root_rot, bone_pos, frames = [20, 40, 60]):
        rot = R.from_quat(root_rot).inv()
        res = []
        for t in frames:
            idx = np.arange(bone_pos.shape[0]) + t
            idx[idx >= bone_pos.shape[0]] = bone_pos.shape[0] - 1
            pos = rot.apply(bone_pos[idx] - root_pos)
            res.append(pos[:,[0,2]])
        return np.concatenate(res, axis = 1)


    def extract_future_rot(self, root_rot, bone_rot, frames = [20, 40, 60]):
        rot = R.from_quat(root_rot).inv()
        res = []
        for t in frames:
            idx = np.arange(bone_rot.shape[0]) + t
            idx[idx >= bone_rot.shape[0]] = bone_rot.shape[0] - 1
            direction = (rot*R.from_quat(bone_rot[idx])).apply(np.array([0,0,1]))
            res.append(direction[:,[0,2]])
        return np.concatenate(res, axis = 1)

class CharacterController:
    def __init__(self, viewer, controller, selected_feature_names, selected_feature_weights):
        self.viewer = viewer
        
        self.motions = []
        self.motions.append(BVHMotion('data/motion_walking_long.bvh'))
        self.motions.append(BVHMotion('data/push.bvh'))
        
        self.db = Database(self.motions, feature_mapping, selected_feature_names, selected_feature_weights)
        
        self.controller = controller
        self.cur_frame = 0
        self.global_bone_pos = np.zeros(3)
        self.global_bone_rot = R.identity()
        self.search_time = 0.1
        self.search_timer = -1
        self.desired_vel_change = np.zeros(3)
        self.desired_rot_change = np.zeros(3)
        
        self.pre_motion_data = None
        self.pre_avel = np.zeros((self.db.joint_num, 3))
        self.rot_offset = np.zeros((self.db.joint_num, 3))
        self.avel_offset = np.zeros((self.db.joint_num, 3))
        
        self.pos_offset = np.zeros((2,3))
        self.vel_offset = np.zeros((2,3))
        self.pre_vel = np.zeros((2,3))

    def update_state(self, desired_pos_list, desired_rot_list, desired_vel_list, desired_avel_list, current_gait):
        
        should_search = False
        if self.search_timer <= 0 or self.cur_frame == self.db.frame_num:
            should_search = True
        eps = 30
        
        if np.linalg.norm(self.desired_vel_change) < eps and np.linalg.norm(self.controller.desired_velocity_change) > eps:
            should_search = True
        
        if np.linalg.norm(self.desired_rot_change) < eps and np.linalg.norm(self.controller.desired_rotation_change) > eps:
            should_search = True
        
        if self.cur_frame == self.db.frame_num - 1:
            should_search = True

        self.desired_vel_change = self.controller.desired_velocity_change
        self.desired_rot_change = self.controller.desired_rotation_change

        if should_search:
            cur_feature = self.db.features[self.cur_frame]
            query_feature = cur_feature.copy()
            
            if 'trajectoryPos2D' in self.db.feature_names:
                future_pos = np.concatenate([np.array(i).reshape(1,3) for i in desired_pos_list[1:4]], axis=0)
                future_pos = self.global_bone_rot.inv().apply(future_pos - self.global_bone_pos.reshape(1,3))
                start_idx = self.db.feature_start_idxs[self.db.feature_names.index('trajectoryPos2D')]
                end_idx = start_idx + self.db.feature_dims[self.db.feature_names.index('trajectoryPos2D')]
                query_feature[start_idx:end_idx] = (future_pos[:,[0,2]]).flatten()
            
            if 'trajectoryRot2D' in self.db.feature_names:
                future_rot = np.concatenate([np.array(i).reshape(1,4) for i in desired_rot_list[1:4]], axis=0)
                future_rot = (self.global_bone_rot.inv() * R.from_quat(future_rot[:,[1,2,3,0]])).apply(np.array([0,0,1]))
                start_idx = self.db.feature_start_idxs[self.db.feature_names.index('trajectoryRot2D')]
                end_idx = start_idx + self.db.feature_dims[self.db.feature_names.index('trajectoryRot2D')]
                query_feature[start_idx:end_idx] = (future_rot[:,[0,2]]).flatten()
            
            normalized_query = self.db.normalize_features(query_feature)
            
            # Do the query
            idx = self.db.query_tree.query(normalized_query.reshape(1,-1), k=1)[1][0]
            
            self.search_timer = self.search_time
            if self.pre_motion_data is not None:
                target_rot = self.db.motion_data[idx].reshape(-1,3).copy()
                target_rot[1] = R.from_quat(desired_rot_list[0]).as_euler('XYZ', degrees=True)
                target_avel = self.db.local_a_velocities[idx].reshape(-1,3).copy()
                self.rot_offset, self.avel_offset = InterpolationHelper.inertialize_transition_rot(
                    self.rot_offset, self.avel_offset, 
                    self.pre_motion_data, self.pre_avel, 
                    target_rot, target_avel)
                
                target_pos = np.zeros_like(self.pos_offset)
                target_pos[0] = desired_pos_list[0]
                target_pos[0] = lerp( self.global_bone_pos , desired_pos_list[0], 0.5)
                target_pos[1] = self.db.motion_data[idx][6:9].copy()
                target_vel = self.db.local_velocities[idx].reshape(-1,3)[[0,2]].copy()
                target_vel[0] = desired_vel_list[0]
                target_vel[1] = 0
                self.pos_offset, self.vel_offset = InterpolationHelper.inertialize_transition_pos(
                    self.pos_offset, self.vel_offset,
                    self.pre_motion_data[[0,2]], self.pre_vel,
                    target_pos, target_vel)
        else:
            idx = self.cur_frame
            self.search_timer -= self.db.dt
        
        idx += 1
        if idx >= self.db.motion_data.shape[0]:
            idx = self.cur_frame -1
            
        motion_data = self.db.motion_data[idx].copy()
        
        if self.pre_motion_data is not None:
            target_rot = self.db.motion_data[idx].reshape(-1,3).copy()
            target_rot[1] = R.from_quat(desired_rot_list[0]).as_euler('XYZ', degrees=True)
            target_avel = self.db.local_a_velocities[idx].reshape(-1,3).copy()
            target_avel[0] = desired_avel_list[1]
            rot, self.pre_avel, self.rot_offset, self.avel_offset = InterpolationHelper.inertialize_update_rot(
                    self.rot_offset, self.avel_offset, target_rot,
                    target_avel,
                    0.05, 1/60)
            
            target_pos = np.zeros_like(self.pos_offset)
            target_pos[0] = lerp( self.global_bone_pos , self.controller.current_desired_position, 0.9999)
            target_pos[1] = self.db.motion_data[idx][6:9]
            target_vel = self.db.local_velocities[idx].reshape(-1,3)[[0,2]].copy()
            target_vel[0] = self.controller.vel
            target_vel[1] = 0
            pos, self.pre_vel, self.pos_offset, self.vel_offset = InterpolationHelper.inertialize_update_pos(
                    self.pos_offset, self.vel_offset,
                    target_pos, target_vel,
                    0.05, 1/60)
            
            motion_data = rot.flatten()
            motion_data[0:3] = pos[0]
            motion_data[6:9] = pos[1]
        else:
            motion_data[0:3] = self.global_bone_pos
            motion_data[3:6] = self.global_bone_rot.as_euler('XYZ', degrees=True)
        
        self.global_bone_pos = motion_data[0:3]
        self.global_bone_rot = R.from_euler('XYZ', motion_data[3:6], degrees=True)
        self.pre_motion_data = motion_data.reshape(-1,3)
        
        joint_translation, joint_orientation = forward_kinematics_with_channel(self.db.joint_parent, self.db.joint_channel, self.db.joint_offset, motion_data.reshape(1,-1))
        joint_translation, joint_orientation = joint_translation[0], joint_orientation[0]
        for name, p, r in zip(self.db.joint_name, joint_translation, joint_orientation):
            self.viewer.set_joint_position_orientation(name, p, r)
        self.cur_frame = idx
        return 

    def sync_controller_and_character(self, controller, character_state):
        self.cur_root_pos = character_state[1][0]
        self.cur_root_rot = character_state[2][0]
        controller.set_pos(self.cur_root_pos)
        controller.set_rot(self.cur_root_rot)
        
        character_state = (self.db.joint_name, character_state[1], character_state[2])
        return character_state

    
class InteractiveUpdate:
    def __init__(self, viewer, controller, character_controller):
        self.viewer = viewer
        self.controller = controller
        self.character_controller = character_controller

    def update(self, task):
        desired_pos_list, desired_rot_list, desired_vel_list, desired_avel_list, current_gait = self.controller.get_desired_state()
        self.character_controller.update_state(
            desired_pos_list, desired_rot_list,
            desired_vel_list, desired_avel_list, current_gait
        )
        return task.cont


def main():
    viewer = SimpleViewer()
    controller = Controller(viewer)
    
    selected_feature_names = ['trajectoryPos2D', 'trajectoryRot2D']
    selected_feature_weights = [1, 1]
    
    # selected_feature_names = ['lFootPos', 'rFootPos']
    # selected_feature_weights = [1, 1]
    
    assert len(selected_feature_names) == len(selected_feature_weights)
    
    character_controller = CharacterController(viewer, controller, selected_feature_names, selected_feature_weights)
    task = InteractiveUpdate(viewer, controller, character_controller)
    viewer.addTask(task.update)
    viewer.run()
    pass


if __name__ == '__main__':
    main()
