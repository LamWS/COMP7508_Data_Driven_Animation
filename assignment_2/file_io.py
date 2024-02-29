import copy
import numpy as np
import scipy.signal as signal
from scipy.spatial.transform import Rotation as R

def load_meta_data(bvh_path):
    with open(bvh_path, 'r') as f:
        channels = []
        joint_names = []
        joint_parents = []
        joint_offsets = []
        end_sites = []

        parent_stack = [None]
        for line in f:
            if 'ROOT' in line or 'JOINT' in line:
                joint_names.append(line.split()[-1])
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif 'End Site' in line:
                end_sites.append(len(joint_names))
                joint_names.append(parent_stack[-1] + '_end')
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif '{' in line:
                parent_stack.append(joint_names[-1])

            elif '}' in line:
                parent_stack.pop()

            elif 'OFFSET' in line:
                joint_offsets[-1] = np.array([float(x) for x in line.split()[-3:]]).reshape(1,3)

            elif 'CHANNELS' in line:
                trans_order = []
                rot_order = []
                for token in line.split():
                    if 'position' in token:
                        trans_order.append(token[0])

                    if 'rotation' in token:
                        rot_order.append(token[0])

                channels[-1] = ''.join(trans_order)+ ''.join(rot_order)

            elif 'Frame Time:' in line:
                break
        
    joint_parents = [-1]+ [joint_names.index(i) for i in joint_parents[1:]]
    channels = [len(i) for i in channels]
    return joint_names, joint_parents, channels, joint_offsets


def load_motion_data(bvh_file_path):
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    
    joint_names, joint_parents, channels, joint_offsets = load_meta_data(bvh_file_path)
    joint_number = len(joint_names)
    
    local_joint_positions = np.zeros((motion_data.shape[0], joint_number, 3))
    local_joint_rotations = np.zeros((motion_data.shape[0], joint_number, 4))
    local_joint_rotations[:,:,3] = 1.0

    cur_channel = 0
    for i in range(len(joint_names)):
        if channels[i] == 0:
            local_joint_positions[:,i,:] = joint_offsets[i].reshape(1,3)
            continue   
        elif channels[i] == 3:
            local_joint_positions[:,i,:] = joint_offsets[i].reshape(1,3)
            rotation = motion_data[:, cur_channel:cur_channel+3]
        elif channels[i] == 6:
            local_joint_positions[:, i, :] = motion_data[:, cur_channel:cur_channel+3]
            rotation = motion_data[:, cur_channel+3:cur_channel+6]
        local_joint_rotations[:, i, :] = R.from_euler('XYZ', rotation,degrees=True).as_quat()
        cur_channel += channels[i]

    return motion_data, local_joint_positions, local_joint_rotations


class BVHMotion():
    def __init__(self, bvh_file_name = None) -> None:
        
        self.joint_name = []
        self.joint_channel = []
        self.joint_parent = []
        
        self.local_joint_positions = None # (N,M,3) 的ndarray, 局部平移
        self.local_joint_rotations = None # (N,M,4)的ndarray, 用四元数表示的局部旋转
        
        if bvh_file_name is not None:
            self.load_motion(bvh_file_name)
        pass
    
    def load_motion(self, bvh_file_path):
        self.joint_name, self.joint_parent, self.joint_channel, self.joint_offset = \
            load_meta_data(bvh_file_path)
        
        self.motion_data, self.local_joint_positions, self.local_joint_rotations = load_motion_data(bvh_file_path)

    def batch_forward_kinematics(self, joint_position=None, joint_rotation=None):
        if joint_position is None:
            joint_position = self.local_joint_positions
        if joint_rotation is None:
            joint_rotation = self.local_joint_rotations
        
        if len(joint_position.shape) == 2:
            joint_position = joint_position.reshape(1, -1, 3).copy()
            joint_rotation = joint_rotation.reshape(1, -1, 4).copy()
        
        joint_translation = np.zeros_like(joint_position)
        joint_orientation = np.zeros_like(joint_rotation)
        joint_orientation[:,:,3] = 1.0 
        
        for i in range(len(self.joint_name)):
            pi = self.joint_parent[i]
            parent_orientation = R.from_quat(joint_orientation[:,pi,:]) 
            joint_translation[:, i, :] = joint_translation[:, pi, :] + \
                parent_orientation.apply(joint_position[:, i, :])
            joint_orientation[:, i, :] = (parent_orientation * R.from_quat(joint_rotation[:, i, :])).as_quat()
        return joint_translation, joint_orientation
    
    def raw_copy(self):
        return copy.deepcopy(self)
    
    def sub_sequence(self, start, end):
        res = self.raw_copy()
        res.local_joint_positions = res.local_joint_positions[start:end,:,:]
        res.local_joint_rotations = res.local_joint_rotations[start:end,:,:]
        return res
    
    def adjust_joint_name(self, target_joint_name):
        idx = [
            self.joint_name.index(joint_name)
            for joint_name in target_joint_name
        ]
        idx_inv = [
            target_joint_name.index(joint_name)
            for joint_name in self.joint_name
        ]
        self.joint_name = [self.joint_name[i] for i in idx]
        self.joint_parent = [idx_inv[self.joint_parent[i]] for i in idx]
        self.joint_parent[0] = -1
        self.joint_channel = [self.joint_channel[i] for i in idx]
        self.local_joint_positions = self.local_joint_positions[:, idx, :]
        self.local_joint_rotations = self.local_joint_rotations[:, idx, :]
        pass