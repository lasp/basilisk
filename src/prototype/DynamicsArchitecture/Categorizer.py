import math


class Node:
    def __init__(self, tag):
        self.tag = tag
        self.adjacentNodeList = []
        self.visited = False

    def __str__(self):
        string = "Node " + self.tag + " has connections "
        for nodeInfo in self.adjacentNodeList:
            string = string + " " + nodeInfo[0].tag + f"-{nodeInfo[1]}"
        return string


def addEdge(node1, node2, weight):
    node1.adjacentNodeList.append([node2, weight])
    node2.adjacentNodeList.append([node1, weight])


def findBase(nodeList):
    for node in nodeList:
        node.visited = False

    for node in nodeList:
        print("### Checking node " + node.tag + " ###")
        if checkBase(node):
            for nodeNew in nodeList:
                nodeNew.visited = False
            return node
        for nodeNew in nodeList:
            nodeNew.visited = False


def checkBase(node):
    node.visited = True
    for adjNodeInfo in node.adjacentNodeList:
        result = adjNodeInfo[1] + search(adjNodeInfo, node)
        if result > 3:
            print("Error: found a long chain")
            return False
        if math.isnan(result):
            return False
    return True


def search(nodeInfo, parentNode):
    print(nodeInfo[0].tag)
    nodeInfo[0].visited = True

    numConnections = len(nodeInfo[0].adjacentNodeList)
    if numConnections > 2:
        print("ERROR: found a fork")
        return math.nan
    elif numConnections == 1:
        return 0

    for adjNodeInfo in nodeInfo[0].adjacentNodeList:
        if adjNodeInfo[0].tag == parentNode.tag:
            continue
        if adjNodeInfo[0].visited:
            print("ERROR: found a loop")
            return math.nan

        return adjNodeInfo[1] + search(adjNodeInfo, nodeInfo[0])


def fork():
    # Make the following graph
    #           A
    #  	       / \
    #         1   1
    # 	     /     \
    # 	    D       B
    #      / \       \
    #     2   1       1
    #    /     \       \
    #   G       E       C
    #            \
    #             1
    #              \
    #               F
    nodeList = [Node("A"), Node("B"), Node("C"), Node("D"), Node("E"), Node("F"), Node("G")]
    addEdge(nodeList[0], nodeList[1], 1)
    addEdge(nodeList[1], nodeList[2], 1)
    addEdge(nodeList[0], nodeList[3], 1)
    addEdge(nodeList[3], nodeList[4], 1)
    addEdge(nodeList[4], nodeList[5], 1)
    addEdge(nodeList[3], nodeList[6], 2)

    print("### Graph structure ###")
    for node in nodeList:
        print(node)

    return nodeList


def longChain():
    # Make the following graph
    # A
    #  \
    #   2
    # 	 \
    # 	  B
    #      \
    #       1
    #        \
    #         C
    #          \
    #           2
    #            \
    #             D
    nodeList = [Node("A"), Node("B"), Node("C"), Node("D")]
    addEdge(nodeList[0], nodeList[1], 2)
    addEdge(nodeList[1], nodeList[2], 1)
    addEdge(nodeList[2], nodeList[3], 2)

    print("### Graph structure ###")
    for node in nodeList:
        print(node)

    return nodeList


def loop():
    # Make the following graph
    # A - 1 - B
    # |       |
    # 1       1
    # |       |
    # C - 1 - D
    #          \
    #           1
    #            \
    #             E
    nodeList = [Node("A"), Node("B"), Node("C"), Node("D"), Node("E")]
    addEdge(nodeList[0], nodeList[1], 1)
    addEdge(nodeList[0], nodeList[2], 1)
    addEdge(nodeList[1], nodeList[3], 1)
    addEdge(nodeList[2], nodeList[3], 1)
    addEdge(nodeList[3], nodeList[4], 1)

    print("### Graph structure ###")
    for node in nodeList:
        print(node)

    return nodeList


def Graph(tag):
    if tag == "fork":
        return fork()
    elif tag == "longChain":
        return longChain()
    elif tag == "loop":
        return loop()


if __name__ == "__main__":
    # Choose the graph to analyze
    graph = Graph("fork")  # fork, longChain, loop

    baseNode = findBase(graph)
    if baseNode is None:
        print("### No Base node found ###")
    else:
        print("### Base node is " + baseNode.tag + " ###")
