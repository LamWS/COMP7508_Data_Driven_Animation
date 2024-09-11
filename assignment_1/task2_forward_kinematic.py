import numpy as np
from scipy.spatial.transform import Rotation as R

import file_io as bvh_reader
from viewer import SimpleViewer


def part1_show_T_pose(viewer, joint_names, joint_parents, joint_offsets):
    '''
    A function to show the T-pose of the skeleton
    joint_names:    Shape - (J)     a list to store the name of each joit
    joint_parents:  Shape - (J)     a list to store the parent index of each joint, -1 means no parent
    joint_offsets:  Shape - (J, 1, 3)  an array to store the local offset to the parent joint
    '''
    global_joint_position = np.zeros((len(joint_names), 3))
    for joint_idx, parent_idx in enumerate(joint_parents):
        '''
            TODO: How to update global_joint_position by OFFSETS?
            Toy Sample: 
                Joint1 in (0, 0)
                the offset between J1 and J2 is (1, 1)
                the offset between J2 and J3 is (1, 1)
                the offset between J3 and J4 is (1, 1)
                Parent -> Childs: J1 -> J2 -> J3 -> J4
                so the global joint position of J4 is (0, 0) + (1, 1) + (1, 1) + (1, 1) = (3, 3)
            Hints: 
                1. There is a joint tree with parent-child relationship (joint_idx, parent_idx)
                2. The OFFSET between joint_idx and parent_idx is known as *joint_offsets[joint_idx]*
                3. The *parents* variable is a topological sort of the skeleton tree
                    * The joints after the current joint MUST be below than the current joint or NO Connection
                    * One iteration on parents variable is enough to cover all joint chains
                4. If parent_idx == -1, then there is no parent joint for current joint, 
                   and the global position of current joint is the same as the local position;
                   else, the current joint global position = the sum of all parent joint offsets
        '''
        ########## Code Start ############


        ########## Code End ############
        viewer.set_joint_position_by_name(joint_names[joint_idx], global_joint_position[joint_idx])

    viewer.run()


def part2_forward_kinametic(viewer, joint_names, joint_parents, joint_offsets, joint_positions, joint_rotations, show_animation=False):
    '''
    A function to calculate the global joint positions and orientations by FK
    F: Frame number;  J: Joint number
   
    joint_names:    Shape - (J)     a list to store the name of each joit
    joint_parents:  Shape - (J)     a list to store the parent index of each joint, -1 means no parent
    joint_offsets:  Shape - (J, 1, 3)  an array to store the local offset to the parent joint
    joint_positions:    Shape - (F, J, 3)   an array to store the local joint positions
    joint_rotations:    Shape - (F, J, 4)   an array to store the local joint rotation in quaternion representation
    '''
    joint_number = len(joint_names)
    frame_number = joint_rotations.shape[0]

    global_joint_positions = np.zeros((frame_number, joint_number, 3))
    global_joint_orientations = np.zeros((frame_number, joint_number, 4))
    global_joint_orientations[:, :, 3] = 1.0

    '''
        TODO: How to update global_joint_position by rotation and offset?
        Sample: 
            Joint1 in (0, 0)
            the offset between J1 and J2 is (1, 0)
            the offset between J2 and J3 is (1, 0)
            then rotate the joint J1 by 45 degree
            then rotate the joint J2 by 45 degree
            How to calculate the global position of J3 after two rotation operations?
            Tips: The results should be (sin45, 1+sin45) 
        Hints: 
            1. There is a joint chain with parent-child relationship (joint_idx, parent_idx)
            2. The OFFSET between joint_idx and parent_idx is known as *joint_offsets[joint_idx]*
            3. The rotation of parent joint will effect all child joints
            4. The *parents* variable is a topological sort of the skeleton
               * The joints after the current joint MUST be below than the current joint
               * One iteration on parents variable is enough to cover all joint chains
        More details:
            1. You can use R.from_quat() to represent a rotation in Scipy format
               like: r1 = R.from_quat(global_joint_orientations[:, joint_idx, :])
            2. Then R.apply() can apply this rotation to any vector
               like: rotated_offset = r1.apply(vector)
            3. new_joint_position = parent_joint_position + parent_joint_rotation.apply(rotated_offset)
               
    '''
    ########## Code Start ############
    

    ########## Code End ############
    if not show_animation:
        show_frame_idx = 0
        viewer.show_pose(
            joint_names, global_joint_positions[show_frame_idx], global_joint_orientations[show_frame_idx])

    else:
        class UpdateHandle:
            def __init__(self):
                self.current_frame = 0

            def update_func(self, viewer_):
                cur_joint_position = global_joint_positions[self.current_frame]
                cur_joint_orentation = global_joint_orientations[self.current_frame]
                viewer.show_pose(
                    joint_names, cur_joint_position, cur_joint_orentation)
                self.current_frame = (self.current_frame + 1) % frame_number

        handle = UpdateHandle()
        viewer.update_func = handle.update_func
    viewer.run()


def main():
    viewer = SimpleViewer()
    bvh_file_path = "data/motion_walking.bvh"

    '''
    Basic data terms in BVH format:
        joint_names:    Shape - (J)     a list to store the name of each joit
        joint_parents:  Shape - (J)     a list to store the parent index of each joint, -1 means no parent
        channels:       Shape - (J)     a list to store the channel number of each joint
        joint_offsets:  Shape - (J, 1, 3)  an array to store the local offset to the parent joint
        local_joint_positions:    Shape - (F, J, 3)   an array to store the local joint positions
        local_joint_rotations:    Shape - (F, J, 4)   an array to store the local joint rotation in quaternion representation
    '''
    joint_names, joint_parents, channels, joint_offsets = bvh_reader.load_meta_data(bvh_file_path)
    _, local_joint_positions, local_joint_rotations = bvh_reader.load_motion_data(bvh_file_path)

    # part 1
    part1_show_T_pose(viewer, joint_names, joint_parents, joint_offsets)

    # part 2
    # part2_forward_kinametic(viewer, joint_names, joint_parents, joint_offsets, local_joint_positions, local_joint_rotations, show_animation=True)


if __name__ == "__main__":
    main()
