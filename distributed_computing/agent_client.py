'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

import threading
#import websocket
import requests

import xmlrpc.client

import sys
sys.path.insert(0, '../')
from joint_control.keyframes import *
import logging


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        #self.server.register_function(self.get_angle, "get_angle")

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE


        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()



        #threading.Thread(target=self.proxy)


    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE

        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform)).start()

        #threading.Thread

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)

        #self.ws = websocket.WebSocket()
        #self.ws.connect("ws://localhost:8000")

        #self.server_url = "http://localhost:8000"

        self.server = xmlrpc.client.ServerProxy("http://localhost:8001")

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE

        #response = requests.get(self.server_url + "/get_angle", params={"joint_name": joint_name})
        #response.raise_for_status()
        #return response.json()
        '''''
        try:
            #response = requests.get(self.server_url + "/get_angle", params={"joint_name": joint_name})
            response = requests.get(self.server_url + "/get_angle", params={"joint_name": joint_name})
            response.raise_for_status()
            return response.json()  # Assuming the server response is JSON
        except Exception as e:
            return("Error in get_angle:", e)
        '''''



        try:
            return self.server.rpc_get_angle(joint_name)
        except Exception as e:
            logging.error("Error in get_angle: %s", e)


    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE

        try:
            return self.server.rpc_set_angle(joint_name, angle)
        except Exception as e:
            logging.error("Error in set_angle: %s", e)


    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        '''''
        try:
            response = requests.get(self.server_url + "/get_posture")
            response.raise_for_status()
            return response.json()  # Assuming the server response is JSON
        except Exception as e:
            return("Error in get_posture:", e)
        '''''


        try:
            return self.server.rpc_get_posture()
        except Exception as e:
             logging.error("Error in get_posture: %s", e)


    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        try:
            return self.server.rpc_execute_keyframes(keyframes)
        except Exception as e:
            logging.error("Error in execute_keyframes: %s", e)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        '''''
        try:
            response = requests.get(self.server_url + "/get_transform", params={"name": name})
            response.raise_for_status()
            return response.json()  # Assuming the server response is JSON
        except Exception as e:
            print("Error in get_transform:", e)
        '''''

        try:
            return self.server.rpc_get_transform(name)
        except Exception as e:
            #print("Error in get_transform")
            logging.error("Error in get_transform: %s", e)



    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        '''''
        try:
            data = {'effector_name': effector_name, 'transform': transform}
            response = requests.post(self.server_url + "/set_transform", json=data)
            response.raise_for_status()
        except Exception as e:
            print("Error in set_transform:", e)
        '''''
        try:
            return self.server.rpc_set_transform(effector_name, transform)
        except Exception as e:
            logging.error("Error in set_transform: %s",e)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print(agent.get_angle("HeadYaw"))
    print(agent.get_posture())
    print(agent.get_transform("HeadYaw"))
    keyframes = hello()
    print("Executing keyframe hello")
    a = agent.execute_keyframes(keyframes)
    print(a)
    a = agent.execute_keyframes(keyframes)
    print(a)
