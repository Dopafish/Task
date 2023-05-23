'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead




class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.starttime = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        target_joints["RHipYawPitch"] = target_joints.get("LHipYawPitch", 0)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):

        target_joints = {}

        # YOUR CODE HERE


        names, times, keys = keyframes

        self.starttime = self.starttime or perception.time

        current_time = perception.time - self.starttime

        for i, joint_name in enumerate(names):
            if joint_name not in self.joint_names:
                continue

            t = None
            for j in range(len(times[i]) - 1):
                if times[i][j] < current_time < times[i][j + 1]:
                    t = (current_time - times[i][j]) / (times[i][j + 1] - times[i][j])
                    break

            if t is not None:
                p_0 = keys[i][j][0]
                p_1 = p_0 + (keys[i][j][2][1] * keys[i][j][2][2])
                p_3 = keys[i][j + 1][0]
                p_2 = p_3 + (keys[i][j + 1][1][1] * keys[i][j + 1][1][2])
                target_joints[joint_name] = (1 - t) ** 3 * p_0 + 3 * (1 - t) ** 2 * t * p_1 + 3 * (1 - t) * t ** 2 * p_2 + t ** 3 * p_3


            elif current_time < times[i][0]:
                p_0 = perception.joint[joint_name]
                p_1 = p_0
                p_3 = keys[i][0][0]
                p_2 = p_3 + (keys[i][0][2][1] * keys[i][0][2][2])
                t = current_time / times[i][0]
                target_joints[joint_name] = (1 - t) ** 3 * p_0 + 3 * (1 - t) ** 2 * t * p_1 + 3 * (1 - t) * t ** 2 * p_2 + t ** 3 * p_3






        # Check status of LHipYawPitch
        # if 'LHipYawPitch' not in target_joints:
            # target_joints['LHipYawPitch'] = perception.joint.get('LHipYawPitch', 0)

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
