from typing import Any, Callable, Dict, List, Tuple

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
                # if type(target_msg) != type(source_msg):
                #     raise Exception(f"source message type {type(source_msg)} != target message type {type(target_msg)}")
                
                msg = type(source_msg)()
                source_msg = msg
                target_msg.subscribeTo(msg)
                self.graph[source_name]["pubs"] = msg

                # target_model.__getattribute__(target_msg).subscribeTo(source_model.__getattribute__(source_msg))

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
        
        self.graph[name] = {"model": model, "neighbors": [], "pubs": []}

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

    def get_messages(self, names: List[str] = None) -> Dict[str, List[Tuple[Any]]]:
        """
            Get messages out of nodes specified by names. Return a dictionary
            implementation of the requested subgraph.

            Params
            ------
                * names: list of string names of nodes

            Return
            ------
                * ret_edges: a dictionary keyed by model name with values as all edges
                    out of that node
        """
        ret_edges = {}
        if names is None:
            names = list(self.graph.keys())

        for name in names:
            ret_edges[name] = self.graph[name]["neighbors"]

        return ret_edges
