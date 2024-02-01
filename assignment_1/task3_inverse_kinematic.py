import numpy as np
from scipy.spatial.transform import Rotation as R

from viewer import SimpleViewer


def norm(v):
    return v/np.sqrt(np.vdot(v, v))


class MetaData:
    '''
        A helper class to store meta data of the skeleton
        get_path_from_root_to_end: return the path from root to end joint
        return:
            path: a list of joint indices from root to end joint
            path_name: a list of joint names from root to end joint
            path1: a list of joint indices from FOOT to ROOT joint
            path2: a list of joint indices from ROOT to End joint
    '''
    def __init__(self, joint_name, joint_parent, joint_initial_position, root_joint, end_joint):
        self.joint_name = joint_name
        self.joint_parent = joint_parent
        self.joint_initial_position = joint_initial_position
        self.root_joint = root_joint
        self.end_joint = end_joint

    def get_path_from_root_to_end(self):

        path1 = [self.joint_name.index(self.end_joint)]
        while self.joint_parent[path1[-1]] != -1:
            path1.append(self.joint_parent[path1[-1]])

        path2 = [self.joint_name.index(self.root_joint)]
        while self.joint_parent[path2[-1]] != -1:
            path2.append(self.joint_parent[path2[-1]])

        while path1 and path2 and path2[-1] == path1[-1]:
            path1.pop()
            a = path2.pop()

        path2.append(a)
        path = path2 + list(reversed(path1))
        path_name = [self.joint_name[i] for i in path]
        return path, path_name, path1, path2


def inverse_kinematics(meta_data, global_joint_positions, global_joint_orientations, target_pose, method='ccd'):
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()

    '''
        Take out all the joint information on the IK chain
        The IK operation should be done on this chain
        Order: start_joint -> end_joint
        chain_positions: a list of joint positions on the IK chain
        chain_offsets: a list of joint offsets on the IK chain
        chain_orientations: a list of joint orientations on the IK chain
    '''
    chain_positions = [global_joint_positions[joint_idx] for joint_idx in path]
    chain_offsets = [np.array([0., 0., 0.])] + [meta_data.joint_initial_position[path[i + 1]] - meta_data.joint_initial_position[path[i]] for i in range(len(path) - 1)]
    chain_orientations = [R.from_quat(global_joint_orientations[path[i]]) for i in range(len(path) - 1)] + [R.identity()]

    # Feel free to implement any other IK methods, bonus will be given
    if method == 'ccd':
        iteration_num = 20
        end_joint_name = meta_data.end_joint
        end_idx = path_name.index(end_joint_name)
        for _ in range(iteration_num):
            for current_idx in range(end_idx - 1, 0, -1):
                '''
                TODO: How to update chain_orientations by optimizing the chain_positions(CCD)?

                Hints:
                    1. The CCD IK is an iterative algorithm, the loop structure is given
                    2. The CCD IK algorithm is to update the orientation of each joint on the IK chain
                    3. Two vectors are essential for the CCD
                       * The vector from end joint to current joint
                       * The vector from target position to current joint
                    4. The rotation matrix can be obtained by these two vectors
                More details about CCD algorithm can be found in the lecture slides.

                Useful functions:
                    1. norm: normalize a vector
                    2. the position of one joint: chain_positions[current_idx]
                    3. vector_between_J1_J2: get the vector from J1 to J2
                       like: vec_cur2end = norm(j_pos[end_idx] - j_pos[current_idx])
                    4. The angle between two vectors: rot = np.arccos(np.vdot(vec1, vec2)) * Sometimes the rot will be nan, so np.isnan() will be helpful
                    5. The axis of rotation: axis = norm(np.cross(vec1, vec2))
                    6. The rotation matrix: rot_vec = R.from_rotvec(rot * axis)
                    7. Update orientation: new_orien = rot_vec * old_orien
                '''
                
                ########## Code Start ############

                vec_cur2end = norm(chain_positions[end_idx] - chain_positions[current_idx])
                vec_cur2tar = norm(target_pose - chain_positions[current_idx])

                axis = norm(np.cross(vec_cur2end, vec_cur2tar))
                rot = np.arccos(np.vdot(vec_cur2end, vec_cur2tar))

                if np.isnan(rot):
                    continue

                rotate_vector = R.from_rotvec(rot * axis)
                chain_orientations[current_idx] = rotate_vector * chain_orientations[current_idx]
                
                ########## Code End ############

                chain_local_rotations = [chain_orientations[0]] + [chain_orientations[i].inv() * chain_orientations[i + 1] for i in range(len(path) - 1)]
                for j in range(current_idx, end_idx):
                    chain_positions[j + 1] = chain_positions[j] + chain_orientations[j].apply(chain_offsets[j + 1])
                    if j + 1 < end_idx:
                        chain_orientations[j + 1] = chain_orientations[j] * chain_local_rotations[j + 1]
                    else:
                        chain_orientations[j + 1] = chain_orientations[j]

    # Update global joint positions and orientations with optimized chain
    local_joint_rotations = [R.identity()]*len(meta_data.joint_parent)
    for joint_idx, parent_idx in enumerate(meta_data.joint_parent):
        if parent_idx == -1:
            local_joint_rotations[joint_idx] = R.from_quat(global_joint_orientations[joint_idx])
        else:
            local_joint_rotations[joint_idx] = R.from_quat(global_joint_orientations[parent_idx]).inv() * R.from_quat(global_joint_orientations[joint_idx])

    for chain_idx, joint_idx in enumerate(path):
        global_joint_positions[joint_idx] = chain_positions[chain_idx]
        global_joint_orientations[joint_idx] = chain_orientations[chain_idx].as_quat()

    for chain_idx, joint_idx in enumerate(path2[:-1]):
        global_joint_orientations[path2[chain_idx+1]] = chain_orientations[chain_idx].as_quat()
    global_joint_orientations[path2[-1]] = chain_orientations[len(path2) - 1].as_quat()

    for joint_idx, parent_idx in enumerate(meta_data.joint_parent):
        if parent_idx != -1 and meta_data.joint_name[joint_idx] not in path_name:
            parent_orientation = global_joint_orientations[parent_idx]
            original_offset = meta_data.joint_initial_position[joint_idx] - meta_data.joint_initial_position[parent_idx]
            rotated_offset = R.from_quat(parent_orientation).apply(original_offset)
            global_joint_positions[joint_idx] = global_joint_positions[parent_idx] + rotated_offset
            global_joint_orientations[joint_idx] = (R.from_quat(global_joint_orientations[parent_idx]) * local_joint_rotations[joint_idx]).as_quat()

    return global_joint_positions, global_joint_orientations


def IK_example(viewer, target_pos, start_joint, end_joint):
    '''
    A simple example for inverse kinematics
    '''
    viewer.create_marker(target_pos, [1, 0, 0, 1])
    joint_name, joint_parent, joint_initial_position = viewer.get_meta_data()
    meta_data = MetaData(joint_name, joint_parent, joint_initial_position, start_joint, end_joint)
    global_joint_position = viewer.get_joint_positions()
    global_joint_orientation = viewer.get_joint_orientations()

    joint_position, joint_orientation = inverse_kinematics(meta_data, global_joint_position, global_joint_orientation, target_pos)
    viewer.show_pose(joint_name, joint_position, joint_orientation)
    viewer.run()
    pass


def IK_interactive(viewer, target_pos, start_joint, end_joint):
    '''
    A simple interactive example for inverse kinematics
    '''
    marker = viewer.create_marker(target_pos, [1, 0, 0, 1])

    joint_name, joint_parent, joint_initial_position = viewer.get_meta_data()
    meta_data = MetaData(joint_name, joint_parent, joint_initial_position, start_joint, end_joint)
    joint_position = viewer.get_joint_positions()
    joint_orientation = viewer.get_joint_orientations()

    class UpdateHandle:
        def __init__(self, marker, joint_position, joint_orientation):
            self.marker = marker
            self.joint_position = joint_position
            self.joint_orientation = joint_orientation

        def update_func(self, viewer):
            target_pos = np.array(self.marker.getPos())
            self.joint_position, self.joint_orientation = inverse_kinematics(meta_data, self.joint_position, self.joint_orientation, target_pos)
            viewer.show_pose(joint_name, self.joint_position, self.joint_orientation)
    handle = UpdateHandle(marker, joint_position, joint_orientation)
    handle.update_func(viewer)
    viewer.update_marker_func = handle.update_func
    viewer.run()


def main():
    viewer = SimpleViewer()

    '''
    You should try different start and end joints and different target positions
    use WASD to move the control points in interactive mode (click the scene to activate the control points)
    '''
    IK_example(viewer, np.array([0.5, 0.75, 0.5]), 'RootJoint', 'lWrist_end')
    # IK_example(viewer, np.array([0.5, 0.75, 0.5]), 'lToeJoint_end', 'lWrist_end')
    # IK_interactive(viewer, np.array([0.5, 0.75, 0.5]), 'RootJoint', 'lWrist_end')
    # IK_interactive(viewer, np.array([0.5, 0.75, 0.5]), 'lToeJoint_end', 'lWrist_end')
    # IK_interactive(viewer, np.array([0.5, 0.75, 0.5]), 'rToeJoint_end', 'lWrist_end')


if __name__ == "__main__":
    main()
