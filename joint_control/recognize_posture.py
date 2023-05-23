'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle




class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER
        # self.classes = os.listdir('robot_pose_data')

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        joint_names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        posture_names = {0: 'Back', 1: 'Belly', 2: 'Crouch', 3: 'Frog',
                         4: 'HeadBack', 5: 'Knee', 6: 'Left', 7: 'Right',
                         8: 'Sit', 9: 'Stand', 10: 'StandInit'}

        data = [perception.joint[name] for name in joint_names]
        data.extend(perception.imu)


        posture_num = self.posture_classifier.predict([data])[0]

        posture = posture_names.get(posture_num, 'unknown')

        print(posture)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
