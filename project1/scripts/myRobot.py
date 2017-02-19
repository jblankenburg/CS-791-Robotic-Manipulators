import numpy as np

# ------------------------------------
# Code for the robot stufffssss

class MyRobot:

    def __init__(self, filename):
        f = open(filename)
        botSpecs = eval(f.read())
        f.close()
        self.parseDHFile(botSpecs)

    def parseDHFile(self, DHparams):
        # list of 4-tuples (a, alpha, d, theta)
        if type(DHparams) == dict:
            self.setDHFromDict(DHparams)
        elif type(DHparams) == list:
            self.dhParams = DHparams
        else:
            throw("Invalid Format")

        print self.dhParams

        self.numLinks = len(self.dhParams)
        self.q = np.zeros((self.numLinks, 1))

    def setDHFromDict(self, DHdict):
        self.dhParams = []
        for joint in DHdict["Joints"]:
            self.dhParams.append(joint["DH_parameters"])

    def getTranslation(self, i,j):
        return self.getT(i,j)[:-1,-1]

    def getRotationMatrix(self, i,j):
        return self.getT(i,j)[:3,:3]


    # constructA: constructs the transform matrix for the ith joint using
    #             the D-H parameters
    # i: the joint to construct the homogeneous transform for
    # returns the transform from the i-1th to the ith
    def constructA(self, i):
        a, alpha, d ,theta = self.dhParams[i]
        # TODO: complete this function to get a homogeneous transform from 
        #       frame i to i+1 using the D-H params and return it
        A = np.array([ 
            np.cos(theta), -np.sin(theta)*np.cos(alpha),   np.sin(theta)*np.sin(alpha),  a*np.cos(theta),
            np.sin(theta),  np.cos(theta)*np.cos(alpha),  -np.cos(theta)*np.sin(alpha),  a*np.sin(theta),
            0,              np.sin(alpha),                 np.cos(alpha),                d,
            0,              0,                             0,                            1,
            ])
        A = A.reshape(4,4)
        # print(A)

        return A


    # getT: computes a homogeneous transform from the ith frame to the jth frame
    #       by iteratively multiplying each homogeneous transform from i to j
    # i is the index of the starting coordinate frame
    # j is the index of the ending coordinate frame
    # returns: a homogeneous transform matrix, T
    def getT(self, i, j):
        # TODO: complete this function to get a transform from the
        #       ith frame to the jth frame and return it (hint: use constructA above)
        T = self.constructA(i)
        # print(T)
        # print(i)
        # print(i+1)
        # print(j)
        
        for k in range(i+1,j):
            # print('K {}'.format(k))
            T = np.dot(T,self.constructA(k))
            # print(T)
        return T


