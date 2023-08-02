class System:
    def __init__(self):
        self.massSC = None
        self.ISC = None
        self.r_CN = None
        self.rDot_CN = None
        self.r_CN = None
        self.rDot_CB = None

        self.base = None
        self.subsytemList = []

    def propagateDynamics(self):
        for subsystem in self.subsytemList:
            subsystem.updateEffectorMassProps()
            subsystem.computeDerivatives()
            subsystem.updateContributions()
            subsystem.updateEnergyMomContributions()
