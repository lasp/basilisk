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

    def init_models(self, model_names=None):
        """
            Instantiate models. Add them to a dictionary keyed by model name.
            Then subscribe to messages per edges in the graph using the instantiated
            objects from the dictionary. Return the dictionary.
        """
        
        if model_names is None:
            model_names = list(self.graph.keys())

        model_dict = {}
        # initialize all models first
        for model_name in model_names:
            model_dict[model_name] = self.graph[model_name]["model"]()
            
        # go through all neighbors of all instantiated models, and subscribe
        # their messages accordingly
        mods_with_neighbs = {
            x: self.graph[x]["neighbors"] for x in model_names if self.graph[x]["neighbors"]
        }
        for source_name in mods_with_neighbs:
            source_model = model_dict[source_name]
            for target_data in mods_with_neighbs[source_name]:
                target_name = target_data[0]
                source_msg, target_msg = target_data[1]
                target_model = model_dict[target_name]
                target_model.__getattribute__(target_msg).subscribeTo(source_model.__getattribute__(source_msg))

        return model_dict

    def register_model(self, model: Callable, name: str):
        """
            Model registration adds a node to the graph.

            Params
            ------
                * model: 
                * name: 
        """
        if name in self.graph:
            raise Exception(f"model of type {model} with name {name} already exists...")
        
        self.graph[name] = {"model": model, "neighbors": []}

    def register_message(self, source_name: str, target_name: str, message_data: Any):
        """
            Message registration adds and edge between two existing nodes in the graph.

            Params
            ------
                * out_name:
                * in_name:
                * message_data: 
        """
        self.graph[source_name]["neighbors"].append((target_name, message_data))

    def get_models(self, names: List[str] = None) -> Dict[str, Callable]:
        """
            Accessor method for retrieving a node (model) from the graph by name. If no names
            are passed, then return all nodes (models) in the .

            Params
            ------
                * name:

            Return
            ------
                * ret_model:
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
            Get messages into and out of nodes specified by names. Return a dictionary
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
