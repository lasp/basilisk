import os
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import KinematicsEngine


def run():
    return


if __name__ == "__main__":
    run()
