from typing import Any, Callable

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

        # hold on to a list of names of modules that have been accessed. A module can only
        # be accessed once. The idea here being that the graph parametrizes what the user can
        # instantiate and modify. Therefore, a node can only be accessed once, otherwise the
        # parametrization has been compromised.
        self.accessed = []

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
        
        self.graph[name] = {"model": model, "in_nodes": [], "out_nodes": []}

    def register_message(self, out_name: str, in_name: str, message_data: Any):
        """
            Message registration adds and edge between two existing nodes in the graph.

            Params
            ------
                * out_name:
                * in_name:
                * message_data: 
        """
        self.graph[out_name]["out_nodes"].append((in_name, message_data))
        self.graph[in_name]["in_nodes"].append((out_name, message_data))

    def get_model(self, name):
        """
            Accessor method for retrieving a node from the graph by name.

            Params
            ------
                * name:

            Return
            ------
                * ret_model:
        """
        if name not in self.graph:
            raise Exception(f"user requested access to model with name {name} but there is no model registered with that name...")
        
        if name in self.accessed:
            raise Exception(f"user tried to access model with name {name} but that model has already been accessed...")

        ret_model = self.graph[name]["model"]
        self.accessed.append(name)

        return ret_model
