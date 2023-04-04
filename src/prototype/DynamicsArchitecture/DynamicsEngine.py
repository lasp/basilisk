class DynamicsEngine:
    def __init__(self):
        self.systemList = []

    def categorizeSystem(self):
        return

    def propagateStates(self):
        # NEED TO FIGURE OUT SYSTEM INTERACTION

        # Loop through all systems
        for system in self.systemList:
            system.propagateDynamics()

        # where to update kinematics?

    def setEnvironment(self):
        return
