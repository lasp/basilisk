from typing import Any, Callable, Dict, List, Tuple, Union

from Basilisk.architecture import messaging

class ExternalModel:
    """
        A placeholder class for external facing models.
        This is essentially so that we can incorporate the nodes
        in the graph external to Basilisk into the registration and
        initialization pattern for internal models.
    """
    def __init__(self):
        pass

class Registry:
    # cache an instance of the class, return it instead of instantiating
    # a new one if it already exists
    _the_registry = None

    def __new__(cls):
        if cls._the_registry is None:
            cls._the_registry = super().__new__(cls)
        return cls._the_registry

    def __init__(self):
        if hasattr(self, "graph") is False:
            self.graph = {}

    def init_models(self, model_names: List[str] = None) -> Dict[str, Callable]:
        """
            Instantiate models. Add them to a dictionary keyed by model name.
            Then subscribe to messages per edges in the graph using the instantiated
            objects from the dictionary. Return the dictionary.

            Params
            ------
                * model_names: list of names of models we wish to initialize. Optional, defaults to 
                    all models in the graph

            Return
            ------
                * model_dict: dict keyed by model name of the instance of the model with that name
        """
        
        if model_names is None:
            model_names = list(self.graph.keys())

        model_dict = {}
        # instantiate all models first, and replace the reference to the class in the graph
        # with a reference to the instantiated object -- this will modify the node as we go ahead and make
        # modifications to the instantiated model
        for model_name in model_names:
            mod = self.graph[model_name]["model"]()
            self.graph[model_name]["model"] = mod
            model_dict[model_name] = mod
            
        # extract all nodes in the graph that have out edges
        mods_with_neighbs = {
            x: self.graph[x]["neighbors"] for x in model_names if self.graph[x]["neighbors"]
        }

        # subscribe all messages for each node with an out edge in the graph
        for source_name in mods_with_neighbs:
            source_model = model_dict[source_name]

            # to this point, ExternalModel objects are empty so add in attributes based on
            # their pubs, which should have been added as placeholders during registration.
            for att_name in self.graph[source_name]["pubs"]:
                source_model.__setattr__(att_name, self.graph[source_name]["pubs"][att_name])

            for target_data in mods_with_neighbs[source_name]:
                target_name = target_data[0]
                target_model = model_dict[target_name]
                for att_name in self.graph[target_name]["pubs"]:
                    target_model.__setattr__(att_name, self.graph[target_name]["pubs"][att_name])

                source_msg_name, target_msg_name = target_data[1]
                source_msg = source_model.__getattribute__(source_msg_name)
                target_msg = target_model.__getattribute__(target_msg_name)

                # this is essentially checking that the source and target have the same payload type
                if callable(source_msg):
                    a = source_msg.__module__
                else:
                    a = type(source_msg).__module__

                if callable(target_msg):
                    b = target_msg.__module__
                else:
                    b = type(target_msg).__module__

                if a != b:
                    raise Exception(
                        f"source message type {a} != target message type {b}"
                    )
                
                # redirect the out message of the source to a standalone message, and subscribe the in message of the target node to
                # the content of that standalone message. This will allow for external entities, e.g. BlackLion, to access all messages
                # by type and name
                msg = self.graph[source_name]["pubs"].get(source_msg_name)
                # if a message of the desired type already exists, then pop it from the pubs, otherwise create a new one
                if msg is None:
                    msg = type(source_msg)()
                elif type(msg) == type:
                    msg = msg()  # external messages have a class ref, so instantiate in that case.

                # instantiate the target message if we haven't yet
                if type(target_msg) == type:
                    target_msg = target_msg()

                # subscribe the target to the standalone message
                target_msg.subscribeTo(msg)
                # put the message back into the pubs for this source node
                self.graph[source_name]["pubs"][source_msg_name] = msg
                # reassign the message now that we have an additional subscriber
                source_model.__setattr__(source_msg_name, msg)

        return model_dict

    def register_model(self, name: str, model: Callable = None):
        """
            Add a node to the graph holding the model with the desired name

            Params
            ------
                * model: reference to a class that we can instantiate when we want to initialize the node
                * name: desired name of the class instance
        """
        if name in self.graph:
            raise Exception(f"model of type {model} with name {name} already exists...")
        
        # If no class reference for the model is passed, then create an empty ExternalModel
        # container for the node in the graph.
        if model is None:
            model = ExternalModel

        self.graph[name] = {"model": model, "neighbors": [], "pubs": {}}

    def register_message(self, source_name: str, target_name: str, message_data: Tuple[str], message_type: Callable = None):
        """
            Message registration adds and edge between two existing nodes in the graph.

            Params
            ------
                * source_name: name of the registered node in the graph that will be the source of the edge
                * target_name: name of the registered node in the graph that will be the target of the edge
                * message_data: tuple (x, y) where x is the name of the out message attribute on the source node
                    and y is the name in the in message attribute on the target node
                * message_type: class reference to the type of message we want to add to the model object. This is intended
                    to be used only for external messages. Optional, default = None
        """
        source_node = self.graph[source_name]["model"]
        if source_node == ExternalModel:
            if message_type is None:
                raise Exception("source node is ExternalModel, but message_type is None")
            source_msg_name, _ = message_data
            self.graph[source_name]["pubs"][source_msg_name] = message_type

        target_node = self.graph[target_name]["model"]
        if target_node == ExternalModel:
            if message_type is None:
                raise Exception("source node is ExternalModel, but message_type is None")
            _, target_msg_name = message_data
            self.graph[target_name]["pubs"][target_msg_name] = message_type

        self.graph[source_name]["neighbors"].append((target_name, message_data))

    def get_message(self, name: str) -> Dict[str, List[Tuple[Any]]]:
        """
            Collect the message with a given name from a node with a given name.

            Params
            ------
                * names: a string formatted as <model_name>-<message_name> where model_name
                    is the name of the model in the graph that publishes to a message named <message_name>

            Return
            ------
                * ret_msg: the requested message
        """
        model_name, message_name = name.split("-")
        assert model_name in self.graph.keys(), \
            f"""
                There is no model in the graph with name {model_name}
            """
        assert message_name in self.graph[model_name]["pubs"], \
            f"""
                There is no message published by {model_name} named {message_name}
            """

        ret_msg =  self.graph[model_name]["pubs"][message_name]

        return ret_msg
