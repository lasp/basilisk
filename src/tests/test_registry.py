from utilities.registry import Registry
from Basilisk.simulation import spacecraft, simpleNav
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.architecture import messaging

class TestClass:
    testOutMsg = None
    testInMsg = None

    def __init__(self, a=None, b=None):
        self.testOutMsg = a if a is not None else None
        self.testInMsg = b if b is not None else None

class OtherTestClass:
    otherTestOutMsg = None
    otherTestInMsg = None

    def __init__(self, a=None, b=None):
        self.otherTestOutMsg = a if a is not None else None
        self.otherTestInMsg = b if b is not None else None

def test_registry_is_singleton():
    reg1 = Registry()
    reg2 = Registry()

    assert reg1 == reg2

    reg1.graph = {}

def test_register_single_model():
    reg = Registry()

    reg.register_model("testClass", TestClass)

    assert "testClass" in reg.graph.keys(), \
        """
            There is no node by the name 'testClass' in the graph, but a model
            by that name was registered
        """

    assert reg.graph["testClass"]["model"] == TestClass, \
        """
            The model associated to the name 'testClass' is not TestClass, but the model
            was registered with that model type
        """
    
    assert reg.graph["testClass"]["neighbors"] == [], \
        """
            The neighbors associated with the model having name 'testClass' are not empty, but
            there were no neighbors specified in the registration of that model
        """
    
    reg.graph = {}

def test_register_external_model():
    reg = Registry()

    reg.register_model(name="blackLion")

    assert "blackLion" in reg.graph, \
        """
            There should be an external node in the graph with the name 'blackLion',
            but no such node exists.
        """

    assert reg.graph["blackLion"]["model"].__name__ == "ExternalModel", \
        f"""
            The node in the graph by the name of 'blackLion' should be of type 'ExternalModel',
            but is of type {type(type(reg.graph["blackLion"]["model"]).__name__)}
        """
    
    reg.graph = {}

def test_register_external_message():
    reg = Registry()

    reg.register_model("blackLion")
    reg.register_model("testClass", TestClass)
    reg.register_message(
        source_name="blackLion",
        target_name="testClass",
        message_data=("scStateOutMsg", "testInMsg"),
        message_type=messaging.SCStatesMsgPayload
    )

    assert reg.graph["blackLion"]["neighbors"] == [("testClass", ("scStateOutMsg", "testInMsg"))]

    # assert reg.graph["blackLion"]["pubs"]["scStateOutMsg"].__name__ == "SCStatesMsgPayload"

    reg.graph = {}
    
def test_register_single_message():
    reg = Registry()

    reg.register_model("testClass", TestClass)
    reg.register_model("otherTestClass", OtherTestClass)
    reg.register_message(source_name="testClass", target_name="otherTestClass", message_data=("testOutMsg", "otherTestInMsg"))

    assert reg.graph["testClass"]["neighbors"] == [("otherTestClass", ("testOutMsg", "otherTestInMsg"))], \
        f"""
            The neighbors associated to the model with name 'testClass' should consist of a single model 'otherTestClass'
            with message data given by ('testOutMsg', 'otherTestInMsg'), but we found {reg.graph["testClass"]["neighbors"]}
        """
    
    assert reg.graph["otherTestClass"]["neighbors"] == [], \
        f"""
            The neighbors associated to model with name 'otherTestClass' should be empty, since no messages were registered with
            model 'otherTestClass' as a source, but we found {reg.graph["otherTestClass"]["neighbors"]}
        """
    
    reg.graph = {}

def test_init_models_no_messages():
    reg = Registry()

    reg.register_model("testClass", TestClass)
    reg.register_model("otherTestClass", OtherTestClass)

    mods = reg.init_models()

    assert type(mods["testClass"]) == TestClass and type(mods["otherTestClass"]) == OtherTestClass , \
        """
            the models should be instantiated and thus their types should be the respective classes
            passed during the registration phase.
        """

    assert mods["testClass"] == reg.graph["testClass"]["model"] and mods["otherTestClass"] == reg.graph["otherTestClass"]["model"], \
        """
            the models returned from initializing models should agree with the models in the graph of the
            registry
        """
    
    reg.graph = {}

def test_init_models_with_messages():
    """
        Use actual objects from built Basilisk here, so that we don't have to reconstruct the
        entire read/write functor pattern used in messaging just to run a simple test.

        However, it would be nice to have some objects built so that this can be tested locally since
        there is no intrinsic dependency on compiled code for any of this to work...
    """
    reg = Registry()

    reg.register_model(model=spacecraft.Spacecraft, name="scObject")
    reg.register_model(model=simpleNav.SimpleNav, name="simpleNavObj")
    reg.register_message(source_name="scObject", target_name="simpleNavObj", message_data=("scStateOutMsg", "scStateInMsg"))

    mods = reg.init_models()

    assert type(mods["scObject"]) == spacecraft.Spacecraft and type(mods["simpleNavObj"]) == simpleNav.SimpleNav , \
        """
            the models should be instantiated and thus their types should be the respective classes
            passed during the registration phase.
        """
    
    assert mods["simpleNavObj"].scStateInMsg.isLinked(), \
        """
            When the models are instantiated in the call to init_models, all messages should be created
            and subscribed to. Thus both models should be linked (to eachother)
        """

    assert mods["scObject"].scStateOutMsg.isLinked()

    reg.graph = {}

def test_init_external_model():
    reg = Registry()

    reg.register_model(name="blackLion")
    
    mods = reg.init_models()

    assert type(mods["blackLion"]).__name__ == "ExternalModel"

    reg.graph = {}

def test_init_external_model_with_messages():
    reg = Registry()

    reg.register_model(name="blackLion")
    reg.register_model(name="simpleNavObj", model=simpleNav.SimpleNav)
    reg.register_message(
        source_name="blackLion",
        target_name="simpleNavObj",
        message_data=("scStateOutMsg", "scStateInMsg"),
        message_type=messaging.SCStatesMsg
    )

    mods = reg.init_models()

    assert mods["blackLion"].scStateOutMsg.isLinked()

    reg.graph = {}

def test_get_message():
    """
        Register two models, register a message between them, and ensure that when we 
        call get_message with the agreed upon naming convention, we get the expected message
        from the graph back.
    """
    reg = Registry()

    reg.register_model(model=spacecraft.Spacecraft, name="scObject")
    reg.register_model(model=simpleNav.SimpleNav, name="simpleNavObj")
    reg.register_message(source_name="scObject", target_name="simpleNavObj", message_data=("scStateOutMsg", "scStateInMsg"))

    reg.init_models() # this will replace all nodes' models with their class instance

    msg = reg.get_message("scObject-scStateOutMsg")

    assert msg == reg.graph["scObject"]["pubs"]["scStateOutMsg"]

    reg.graph = {}

def test_get_external_message():
    reg = Registry()

    reg.register_model(name="blackLion")
    reg.register_model(name="simpleNavObj", model=simpleNav.SimpleNav)
    reg.register_message(
        source_name="blackLion",
        target_name="simpleNavObj",
        message_data=("scStateOutMsg", "scStateInMsg"),
        message_type=messaging.SCStatesMsg
    )

    mods = reg.init_models()

    msg = reg.get_message("blackLion-scStateOutMsg")

    assert msg == mods["blackLion"].scStateOutMsg

    reg.graph = {}

def test_registry_maintains_graph():
    reg1 = Registry()
    reg1.graph = {"a": [1, 2, 3]}

    reg2 = Registry()

    assert reg2.graph == {"a": [1, 2, 3]}

    reg1.graph = {}


def test_target_external_node():
    reg = Registry()

    reg.register_model(name="blackLion")
    reg.register_model(name="cppModule1", model=cppModuleTemplate.CppModuleTemplate)
    reg.register_message(
        source_name="cppModule1",
        target_name="blackLion",
        message_data=("dataOutMsg", "dataInMsg"),
        message_type=messaging.CModuleTemplateMsgReader
    )

    reg.init_models()

    msg = reg.get_message("cppModule1-dataOutMsg")

    assert msg == reg.graph["cppModule1"]["pubs"]["dataOutMsg"]

    reg.graph = {}
