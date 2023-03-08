from utilities import RigidBodyKinematics as rbk

class DCM:
    def __init__(self, dcm):
        self.dcm = dcm

    def toMRP(self):
        return MRP(rbk.C2MRP(self.dcm))


class Euler321:
    def __init__(self, euler):
        self.euler = euler

    def toMRP(self):
        return MRP(rbk.euler3212MRP(self.euler))


class PRV:
    def __init__(self, prv):
        self.prv = prv

    def toMRP(self):
        return MRP(rbk.PRV2MRP(self.prv))


class Quaternion:
    def __init__(self, quaternion):
        self.quaternion = quaternion

    def toMRP(self):
        return MRP(rbk.EP2MRP(self.quaternion))


class CRP:
    def __init__(self, crp):
        self.crp = crp

    def toMRP(self):
        return MRP(rbk.gibbs2MRP(self.crp))


class MRP:
    def __init__(self, mrp):
        self.mrp = mrp

    def toMRP(self):
        return self
