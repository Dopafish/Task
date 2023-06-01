'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler as RequestHandler

from inverse_kinematics import InverseKinematicsAgent
import json
import threading
import numpy as np
from agent_client import PostHandler


import sys
sys.path.insert(0, '../')
from joint_control.keyframes import *



class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE


    def __init__(self):
        super(ServerAgent, self).__init__()

        self.post = PostHandler(self)
        self.server = SimpleXMLRPCServer(("localhost", 8001))

        self.server.register_function(self.get_angle, "get_angle")
        self.server.register_function(self.set_angle, "set_angle")
        self.server.register_function(self.get_posture, "get_posture")
        self.server.register_function(self.execute_keyframes, "execute_keyframes")
        self.server.register_function(self.get_transform, "get_transform")
        self.server.register_function(self.set_transform, "set_transform")

        try:
            t = threading.Thread(target=self.server.serve_forever)
            t.start()
        except Exception as e:
            print("Error starting server: ", e)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        if joint_name in self.joint_names and joint_name in self.perception.joint:
            return self.perception.joint[joint_name]
        else:
            return "Invalid joint name"
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if joint_name in self.joint_names and joint_name in self.target_joints:
            self.target_joints[joint_name] = angle
        else:
            return "Invalid joint name"

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

        with threading.Lock():
            transform = self.transforms[name].tolist()  # convert the numpy array to a python list
        json_transform = json.dumps(transform)
        return json_transform


    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        if isinstance(transform, str):
            transform = np.array(json.loads(transform))
        elif isinstance(transform, list):
            transform = np.array(transform)
        else:
            return "Invalid transform format"

        if effector_name not in self.joint_names:
            return "Invalid effector name"

        try:
            joint_angles = self.inverse_kinematics(effector_name, transform)
            if joint_angles is None or len(joint_angles) != len(self.joint_names):
                return "Error in inverse kinematics"
            else:
                self.target_joints = dict(zip(self.joint_names, joint_angles))
                return True
        except Exception as e:
            return f"Error in setting transform: {e}"

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

