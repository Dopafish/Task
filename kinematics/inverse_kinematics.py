'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
#from autograd import grad
from numpy.matlib import matrix, linalg, asarray
import numpy as np
from scipy.linalg import pinv
from math import atan2

class InverseKinematicsAgent(ForwardKinematicsAgent):


    def inverse_kinematics(self, effector_name, transform):
    
        
        #solve the inverse kinematics

        #:param str effector_name: name of end effector, e.g. LLeg, RLeg
        #:param transform: 4x4 transform matrix
        #:return: list of joint angles
       

        joint_angles = []

        # YOUR CODE HERE


        target = self._from_transformation_column(transform)
        joint_angles = self._initialize_joint_angles(effector_name)

        l = 0.1  # lambda
        max_step = 0.1  # gradient descent step size
        margin_error = 1e-4  # margin of error
        steps = 1000  # max number of steps

        for i in range(steps):
            joint_angles = self._update_joint_angles(effector_name, joint_angles, target, l, max_step)
            if self._is_converged(joint_angles, margin_error):
                break

        return list(joint_angles.values())

    def _from_transformation_column(self, matrix):

        return [
            matrix[-1, 0],
            matrix[-1, 1],
            matrix[-1, 2],
            atan2(matrix[2, 1], matrix[2, 2]),
        ]

    def _initialize_joint_angles(self, effector_name):
        joint_angles = {}
        for joint in self.chains[effector_name]:
            joint_angles[joint] = self.perception.joint[joint]
        for joint in self.joint_names:
            if joint not in joint_angles:
                joint_angles[joint] = 0
        return joint_angles

    def _update_joint_angles(self, effector_name, joint_angles, target, l, max_step):
        self.forward_kinematics(joint_angles)
        Ts = list(self.transforms.values())
        Te = matrix([self._from_transformation_column(Ts[-1])]).T
        e = target - Te
        e[e > max_step] = max_step
        e[e < -max_step] = -max_step
        T = matrix([self._from_transformation_column(i) for i in Ts[:-1]]).T
        J = Te - T
        dT = Te - T
        J[0, :] = dT[2, :]
        J[1, :] = dT[1, :]
        J[2, :] = dT[0, :]
        J[-1, :] = 1
        d_theta = l * pinv(J).dot(e)
        for i, joint in enumerate(self.chains[effector_name]):
            joint_angles[joint] += np.asarray(d_theta.T)[0][i]
        return joint_angles

    def _is_converged(self, joint_angles, margin_error):
        """Check if the solution has converged"""
        return linalg.norm(list(joint_angles.values())) < margin_error



        #return joint_angles

    def set_transforms(self, effector_name, transform):
        #solve the inverse kinematics and control joints use the results

        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)


        keyframes = self.create_keyframes(effector_name, joint_angles)


        self.keyframes = keyframes

    def create_keyframes(self, effector_name, joint_angles):
        # Get the joint names and initialize keyframe parameters
        joint_names = self.chains[effector_name]
        times = [[2.0, 6.0]] * len(joint_names)
        keys = []


        for i, joint_name in enumerate(joint_names):
            current_angle = self.perception.joint[joint_name]
            target_angle = joint_angles[i]
            key = [[current_angle, [3, 0, 0], [3, 0, 0]], [target_angle, [3, 0, 0], [3, 0, 0]]]
            keys.append(key)

        return (joint_names, times, keys)




        #self.keyframes = ([], [], [])  # the result joint angles have to fill in
    '''''

    def __init__(self, lmbda=0.1, max_step=0.1, margin_error=1e-4, max_iterations=1000):
        super().__init__()
        self.lmbda = lmbda
        self.max_step = max_step
        self.margin_error = margin_error
        self.max_iterations = max_iterations

    def inverse_kinematics(self, effector_name, transform):
        joint_angles = self._initialize_joint_angles(effector_name)
        target_position_orientation = self._get_target_position_orientation(transform)

        for _ in range(self.max_iterations):
            self.forward_kinematics(joint_angles)
            end_effector_pos_orient = self._get_end_effector_pos_orient()

            if self._update_joint_angles(joint_angles, target_position_orientation, end_effector_pos_orient):
                break

        return list(joint_angles.values())

    def set_transforms(self, effector_name, transform):
        joint_angles = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name]
        times = [[2.0, 6.0]] * len(names)
        keys = self._generate_keys(names, joint_angles)

        self.keyframes = (names, times, keys)

    def _initialize_joint_angles(self, effector_name):
        joint_angles = {joint: self.perception.joint[joint] for joint in self.chains[effector_name]}
        joint_angles.update({joint: 0 for joint in self.joint_names if joint not in joint_angles})
        return joint_angles

    @staticmethod
    def _get_target_position_orientation(transform):
        return [
            transform[-1, 0],
            transform[-1, 1],
            transform[-1, 2],
            atan2(transform[2, 1], transform[2, 2]),
        ]

    def _get_end_effector_pos_orient(self):
        transforms = list(self.transforms.values())
        return matrix(self._get_target_position_orientation(transforms[-1])).T

    def _update_joint_angles(self, joint_angles, target, end_effector):
        error = self._get_error(target, end_effector)
        jacobian = self._get_jacobian(end_effector, target)
        angle_updates = self.lmbda * pinv(jacobian).dot(error)

        for idx, joint in enumerate(self.chains[effector_name]):
            joint_angles[joint] += asarray(angle_updates.T)[0][idx]

        return linalg.norm(angle_updates) < self.margin_error

    def _get_error(self, target, end_effector):
        error = target - end_effector
        error[error > self.max_step] = self.max_step
        error[error < -self.max_step] = -self.max_step
        return error
    '''''
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)

    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()


