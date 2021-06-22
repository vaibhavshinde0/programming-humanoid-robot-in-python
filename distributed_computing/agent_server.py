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

import os
import sys
import threading
import xmlrpc.server as SimpleXMLRPCServer
import pickle
import time
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCRequestHandler

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.posture_classifier = 'robot_pose.pkl'
        self.posture = 'unknown'
        
        self.server = SimpleXMLRPCServer.SimpleXMLRPCServer(('localhost', 8888), requestHandler=RequestHandler, allow_none=True)
        self.server.register_introspection_functions()
        self.server.register_multicall_functions()
        self.server.register_instance(self)
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()
        print('RPC Server is ready')

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(ServerAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        Data = [[perception.joint['LHipYawPitch'],
		 perception.joint['LHipRoll'],
		 perception.joint['LHipPitch'],
		 perception.joint['LKneePitch'],
		 perception.joint['RHipYawPitch'],
		 perception.joint['RHipRoll'],
		 perception.joint['RHipPitch'],
		 perception.joint['RKneePitch'],
		 perception.imu[0],
		 perception.imu[1]]]

        name_classes = ['HeadBack', 'Left', 'Right', 'Crouch', 'Knee', 'Stand', 'Sit', 'StandInit', 'Frog', 'Back', 'Belly']
        clf = pickle.load(open(self.posture_classifier))
        predicted = clf.predict(Data)
        posture = name_classes[predicted[0]]

        return posture

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint.get(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = angle
            print('Setting angle ', joint_name, 'to ', str(angle))
        else:
            print('Error: Failed to set angle')

    def get_posture(self):
        '''return current posture of robot'''
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.transforms[effector_name] = transform


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

