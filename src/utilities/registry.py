from typing import Any, Callable, Dict, List, Tuple

from Basilisk.architecture import messaging

class Registry:
    # cache an instance of the class, return it instead of instantiating
    # a new one if it already exists
    _the_registry = None

    def __new__(cls):
        if cls._the_registry is None:
            cls._the_registry = super().__new__(cls)
        return cls._the_registry

    def __init__(self):
        # TODO: make a graph object to abstract this away
        if hasattr(self, "graph") is False:
            self.graph = {}

    def init_models(self, model_names=None) -> Dict[str, Callable]:
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
            
        # extract all nodes in the graph with out edges
        mods_with_neighbs = {
            x: self.graph[x]["neighbors"] for x in model_names if self.graph[x]["neighbors"]
        }

        # subscribe all messages for each node with an out edge in the graph
        for source_name in mods_with_neighbs:
            source_model = model_dict[source_name]
            for target_data in mods_with_neighbs[source_name]:
                target_name = target_data[0]
                source_msg_name, target_msg_name = target_data[1]
                target_model = model_dict[target_name]
                source_msg = source_model.__getattribute__(source_msg_name)
                target_msg = target_model.__getattribute__(target_msg_name)

                # TODO: This still doesn't work correctly. We need to figure out what to do
                #   about models that have vectorized out messages...
                #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<BROKEN>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                # sometimes messages are written out on models as vectors; check for that case,
                # and pick off the last element of the vector
                target_msg_vec = None
                if hasattr(target_msg, "__iter__"):
                    target_msg_vec = target_msg
                    l = target_msg_vec.size()
                    if l == 0:
                        msg_type = type(target_msg_vec).__module__.split(".")[-1].replace("Payload", "")
                        msg_cls = getattr(messaging, msg_type)
                        msg_tmp = msg_cls()
                        target_msg_vec.append(msg_tmp)
                        l += 1

                    target_msg = target_msg_vec[l - 1]

                source_msg_vec = None
                if hasattr(source_msg, "__iter__"):
                    source_msg_vec = source_msg
                    l = source_msg_vec.size()
                    if l == 0:
                        msg_type = type(source_msg_vec).__module__.split(".")[-1].replace("Payload", "")
                        msg_cls = getattr(messaging, msg_type)
                        msg_tmp = msg_cls()
                        target_msg_vec.append(msg_tmp)
                        l += 1

                    source_msg = source_msg_vec[l - 1]
                #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

                # this is essentially checking that the source and target have the same payload type
                if type(target_msg).__module__ != type(source_msg).__module__:
                    raise Exception(
                        f"source message type {type(source_msg.__module__)} != target message type {type(target_msg.__module__)}"
                    )
                
                # redirect the out message of the source to a standalone message, and subscribe the in message of the target node to
                # the content of that standalone message. This will allow for external entities, e.g. BlackLion, to access all messages
                # by type and name
                mask = [type(x).__module__ == type(source_msg).__module__ for x in self.graph[source_name]["pubs"]]
                val = sum(mask)
                msg = self.graph[source_name]["pubs"].get(source_msg_name)
                # if a message of the desired type already exists, then pop it from the pubs, otherwise create a new one
                if msg is None:
                    msg = type(source_msg)()

                # subscribe the target to the standalone message
                target_msg.subscribeTo(msg)
                # put the message back into the pubs for this source node
                self.graph[source_name]["pubs"][source_msg_name] = msg
                # reassign the message now that we have an additional subscriber
                if source_msg_vec is not None:
                    l = source_msg_vec.size()
                    # put the modified message back into the location from which it came
                    source_msg_vec[l - 1] = msg
                    source_model.__setattr__(source_msg_name, source_msg_vec)
                else:
                    source_model.__setattr__(source_msg_name, msg)

        return model_dict

    def register_model(self, model: Callable, name: str):
        """
            Add a node to the graph holding the model with the desired name

            Params
            ------
                * model: reference to a class that we can instantiate when we want to initialize the node
                * name: desired name of the class instance
        """
        if name in self.graph:
            raise Exception(f"model of type {model} with name {name} already exists...")
        
        self.graph[name] = {"model": model, "neighbors": [], "pubs": {}}

    def register_message(self, source_name: str, target_name: str, message_data: Tuple[str]):
        """
            Message registration adds and edge between two existing nodes in the graph.

            Params
            ------
                * source_name: name of the registered node in the graph that will be the source of the edge
                * target_name: name of the registered node in the graph that will be the target of the edge
                * message_data: tuple (x, y) where x is the name of the out message attribute on the source node
                    and y is the name in the in message attribute on the target node
        """
        self.graph[source_name]["neighbors"].append((target_name, message_data))

    def get_models(self, names: List[str] = None) -> Dict[str, Callable]:
        """
            Accessor method for retrieving a node (model) from the graph by name. If no names
            are passed, then return all nodes (models) in the graph.

            Params
            ------
                * names: list of model names. Optional, defaults to all node names in the graph

            Return
            ------
                * ret_models: dict keyed by model names consisting of the "model" value from that node in
                    the graph
        """
        ret_models = {}
        if names is None:
            names = list(self.graph.keys())

        for name in names:
            if name not in self.graph:
                raise Exception(f"user requested access to model with name {name} but there is no model registered with that name...")

            ret_models[name] = self.graph[name]["model"]

        return ret_models

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
