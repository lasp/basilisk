import sys

# append current directory so that python can find local modules
sys.path.append(".")

from src.utilities.registry import Registry

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import spacecraft, simpleNav
from Basilisk.architecture import messaging

reg = Registry()

def register():
    reg.register_model(model=spacecraft.Spacecraft, name="scObject")
    reg.register_model(model=simpleNav.SimpleNav, name="simpleNavObj")
    reg.register_message(source_name="scObject", target_name="simpleNavObj", message_data=("scStateOutMsg", "scStateInMsg"))

def run_maybe():
    # This:
    #   * instantiates all models registered in the register() function
    #   * subscribes all messages registered above
    #   * returns a dict keyed by model name of the instantiated models.
    models = reg.init_models()

    scObject = models["scObject"]
    scObject.ModelTag = "spacecraftBody"

    simpleNavObj = models["simpleNavObj"]
    simpleNavObj.ModelTag = "simpleNavObject"

def run():
    # get all registered models from the registry
    models = reg.get_models()

    # Thought: We can add a model_init and message_init function to
    #   the registry to hide away the details of how this works. The user
    #   still needs access to the model variables locally though so...
    #   Also, once models are initialized we should replace the value
    #   of the node in the graph with the initialized model so that all
    #   updates to it are reflected in the nodes in the graph...

    # replace model instantiation with a call to registry
    # scObject = spacecraft.Spacecraft()
    scObject = models["scObject"]()
    scObject.ModelTag = "spacecraftBody"

    # simpleNavObj = simpleNav.SimpleNav()
    simpleNavObj = models["simpleNavObj"]()
    simpleNavObj.ModelTag = "simpleNavObject"
    
    msg = reg.get_messages(names=["scObject"])["scObject"][0]
    scMsgName = msg[1][0]
    simpleNavMsgName = msg[1][1]

    # Question: how to make this not so ugly for the user...?
    simpleNavObj.__getattribute__(simpleNavMsgName).subscribeTo(scObject.__getattribute__(scMsgName))
    simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

if __name__ == "__main__":
    register()
    run_maybe()
    # run()