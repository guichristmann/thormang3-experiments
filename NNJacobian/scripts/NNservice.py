#!/usr/bin/env python2

from keras.models import load_model
import rospy
from NNJacobian.srv import *
import numpy as np

    
model_fn = "/home/ntnuerc/catkin_ws/src/NNJacobian/scripts/NNmodel.h5"
model = load_model(model_fn)
model.summary()
print("test")
model.predict(np.zeros((1,13)))
print("ok")

# Copied from the C++ code to generate the data
orig_lower = np.array([-1.8393, 0.7837, -0.4403, -1.2003, 0.0514, -0.4201, -1.5500])
orig_upper = np.array([0.5173, 2.2218, 0.8960, 1.1748, 1.8641, 0.9655, 0.3806])

norm_upper = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
norm_lower = np.array([-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5])

# Ignore a and b for compatiblity purposes
def normalizeArray(data):
    # Normalizing Joint data. Should be between limits used for generation
    for i, c in enumerate(range(6, 13, 1)):
        old_range = orig_upper[i] - orig_lower[i]
        new_range = norm_upper[i] - norm_lower[i]

        data[c] = ((data[c] - orig_lower[i]) * new_range / old_range) + norm_lower[i]

    return data


def run_NN(req):
    predictArray = [    req.a, #x
                        req.b, #y
                        req.c, #z
                        req.d, #r
                        req.e, #p
                        req.f, #y
                        req.g,
                        req.h,
                        req.i,
                        req.j,
                        req.k,
                        req.l,
                        req.m ]
                                
    predictArray = np.array(normalizeArray(predictArray),dtype = np.float64)
    deltaAngles = np.array(model.predict(predictArray.reshape(1,13)))[0]

    
    response = GetNNDeltaResponse()
    response.dj0 = deltaAngles[0]
    response.dj1 = deltaAngles[1]
    response.dj2 = deltaAngles[2]
    response.dj3 = deltaAngles[3]
    response.dj4 = deltaAngles[4]
    response.dj5 = deltaAngles[5]
    response.dj6 = deltaAngles[6]
    return response
    
    
if __name__ == "__main__":
    rospy.init_node("NNserviceNode")
    s = rospy.Service("NNService", GetNNDelta, run_NN)
    
    print("Service running.")
    rospy.spin()
   
