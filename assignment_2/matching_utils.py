import math
import numpy as np
from scipy.spatial.transform import Rotation as R        
        
def from_euler(e):
    return R.from_euler('XYZ', e, degrees=True)
        
def lerp(a, b, t):
    return a + (b - a) * t


def align_quat(qt: np.ndarray, inplace: bool):
    ''' make q_n and q_n+1 in the same semisphere

        the first axis of qt should be the time
    '''
    qt = np.asarray(qt)
    if qt.shape[-1] != 4:
        raise ValueError('qt has to be an array of quaterions')

    if not inplace:
        qt = qt.copy()

    if qt.size == 4:  # do nothing since there is only one quation
        return qt

    sign = np.sum(qt[:-1] * qt[1:], axis=-1)
    sign[sign < 0] = -1
    sign[sign >= 0] = 1
    sign = np.cumprod(sign, axis=0, )

    qt[1:][sign < 0] *= -1

    return qt


def quat_to_avel(rot, dt):
    quat_diff = (rot[1:] - rot[:-1])/dt
    quat_diff[...,-1] = (1 - np.sum(quat_diff[...,:-1]**2, axis=-1)).clip(min = 0)**0.5
    quat_tmp = rot[:-1].copy()
    quat_tmp[...,:3] *= -1
    shape = quat_diff.shape[:-1]
    rot_tmp = R.from_quat(quat_tmp.reshape(-1, 4)) * R.from_quat(quat_diff.reshape(-1, 4))
    return 2 * rot_tmp.as_quat().reshape( shape + (4, ) )[...,:3]


def forward_kinematics_with_channel(joint_parent, joint_channel, joint_offset, motion_data):
    num = len(joint_parent)
    joint_positions = np.zeros((motion_data.shape[0], num, 3))
    joint_orientations = np.zeros((motion_data.shape[0], num, 4))
    joint_orientations[...,3] = 1
    
    j = 0
    for i in range(num):
        
        if joint_channel[i] == 6:
            joint_positions[:, i] = joint_offset[i] + motion_data[:, j:j+3]
            rot = R.from_euler('XYZ', motion_data[:, j+3:j+6], degrees=True)
            if joint_parent[i] != -1:
                joint_positions[:, i] += joint_positions[:, joint_parent[i]]
                rot = R.from_quat(joint_orientations[:, joint_parent[i]])*rot
            joint_orientations[:, i] = rot.as_quat()
            j += 6
        else:
            parent_rot = R.from_quat(joint_orientations[:, joint_parent[i]])
            joint_positions[:, i] = joint_positions[:, joint_parent[i]] + parent_rot.apply(joint_offset[i])
            if joint_channel[i] == 3:
                joint_orientations[:, i] = (parent_rot*R.from_euler('XYZ', motion_data[:, j:j+3], degrees=True)).as_quat()
                j += 3
    return joint_positions, joint_orientations


class InterpolationHelper():
    
    @staticmethod
    def halflife2dampling(halflife):
        return 4 * math.log(2) / halflife

    def simulation_positions_update(pos, vel, acc, target_vel, halflife, dt):
        d = InterpolationHelper.halflife2dampling(halflife)/2
        j0 = vel - target_vel
        j1 = acc + d * j0
        eydt = math.exp(-d * dt)
        pos_prev = pos
        tmp1 = j0+j1*dt
        tmp2 = j1/(d*d)
        pos = eydt * ( -tmp2 -tmp1/d ) + tmp2 + j0/d + target_vel*dt + pos_prev
        vel = eydt*tmp1 + target_vel
        acc = eydt * (acc - j1*d*dt)
        return pos, vel, acc
    
    @staticmethod
    def simulation_rotations_update(rot, avel, target_rot, halflife, dt):
        d = InterpolationHelper.halflife2dampling(halflife)/2
        j0 = R.from_quat(rot) * R.from_quat(target_rot).inv()
        j0 = j0.as_rotvec()
        j1 = avel + d * j0
        eydt = math.exp(-d * dt)
        tmp1 = eydt * (j0 + j1 * dt)
        rot = R.from_rotvec(tmp1) * R.from_quat(target_rot)
        rot = rot.as_quat()
        avel = eydt * (avel - j1 * dt * d)
        return rot, avel
    
    @staticmethod
    def decay_spring_implicit_damping_rot(rot, avel, halflife, dt):
        d = InterpolationHelper.halflife2dampling(halflife)/2
        j0 = from_euler(rot).as_rotvec()
        j1 = avel + d * j0
        eydt = math.exp(-d * dt)
        a1 = eydt * (j0+j1*dt)
       
        rot_res = R.from_rotvec(a1).as_euler('XYZ', degrees=True)
        avel_res = eydt * (avel - j1 * dt * d)
        return rot_res, avel_res
    
    @staticmethod
    def decay_spring_implicit_damping_pos(pos, vel, halflife, dt):
        d = InterpolationHelper.halflife2dampling(halflife)/2
        j1 = vel + d * pos
        eydt = math.exp(-d * dt)
        pos = eydt * (pos+j1*dt)
        vel = eydt * (vel - j1 * dt * d)
        return pos, vel
    
    @staticmethod
    def inertialize_transition_rot(prev_off_rot, prev_off_avel, src_rot, src_avel, dst_rot, dst_avel):
        prev_off_rot, prev_off_avel = InterpolationHelper.decay_spring_implicit_damping_rot(prev_off_rot, prev_off_avel, 1/20, 1/60)
        off_rot = from_euler(prev_off_rot) * from_euler(src_rot) * from_euler(dst_rot).inv()
        off_avel = prev_off_avel + src_avel - dst_avel
        # off_rot = from_euler(src_rot) * from_euler(dst_rot).inv()
        # off_avel = src_avel - dst_avel
        return off_rot.as_euler('XYZ', degrees=True), off_avel
    
    @staticmethod
    def inertialize_update_rot(prev_off_rot, prev_off_avel, rot, avel, halflife, dt):
        off_rot , off_avel = InterpolationHelper.decay_spring_implicit_damping_rot(prev_off_rot, prev_off_avel, halflife, dt)
        rot = from_euler(off_rot) * from_euler(rot)
        avel = off_avel + avel
        return rot.as_euler('XYZ', degrees=True), avel, off_rot, off_avel
    
    @staticmethod
    def inertialize_transition_pos(prev_off_pos, prev_off_vel, src_pos, src_vel, dst_pos, dst_vel):
        prev_off_pos, prev_off_vel = InterpolationHelper.decay_spring_implicit_damping_pos(prev_off_pos, prev_off_vel, 1/20, 1/60)
        off_pos = prev_off_pos + src_pos - dst_pos
        off_vel = prev_off_vel + src_vel - dst_vel
        return off_pos, off_vel
    
    @staticmethod
    def inertialize_update_pos(prev_off_pos, prev_off_vel, pos, vel, halflife, dt):
        off_pos , off_vel = InterpolationHelper.decay_spring_implicit_damping_pos(prev_off_pos, prev_off_vel, halflife, dt)
        pos = off_pos + pos
        vel = off_vel + vel
        return pos, vel, off_pos, off_vel
    