class Graph:
    def __init__(self):
        return


class Node:
    def __init__(self, parentNode, frame):
        self.childrenNodeList = []
        self.parentNode = parentNode
        if parentNode is not None:
            self.parentNode.childrenNodeList.append(self)

        self.frame = frame


# Helper function to find the LCA
def findLCA(path1, path2):
    ind = 0
    # Now iterate over the path list found.
    while ind < len(path1) and ind < len(path2):
        # If there is a mismatch break the loop.
        if path1[ind] != path2[ind]:
            break
        ind += 1

    # Return the node encountered just before
    # the mismatch.
    return path1[ind-1:], path2[ind-1:]


def findPath(node, frame, path):
    path.append(node.frame)

    if node.frame == frame:
        return True

    for childNode in node.childrenNodeList:
        if findPath(childNode, frame, path):
            return True

    path.pop()

    # Returning false if nothing worked out.
    return False


def findPathAlternate(frame, path):
    newFrame = frame
    while newFrame is not None:
        path.append(newFrame)
        newFrame = newFrame.parentFrame

    path.reverse()


if __name__ == "__main__":
    # Making the following tree
    #       1
    #  	   /|\
    # 	  / | \
    # 	 2  3  4
    #   / \     \
    #  /   \     \
    # 5     6     7
    #      /|\
    #     / | \
    #    8  9  10
    node1 = Node(None, 1)
    node2 = Node(node1, 2)
    node3 = Node(node1, 3)
    node4 = Node(node1, 4)
    node5 = Node(node2, 5)
    node6 = Node(node2, 6)
    node7 = Node(node4, 7)
    node8 = Node(node6, 8)
    node9 = Node(node6, 9)
    node10 = Node(node6, 10)

    # Choose the nodes
    frame1 = 5
    frame2 = 8

    # Find the path
    path1 = []
    path2 = []
    findPath(node1, frame1, path1)
    findPath(node1, frame2, path2)
    print(f'Path from node {frame1} to base node: {path1}')
    print(f'Path from node {frame2} to base node: {path2}')

    # Find the common ancestor between two frames and return path to it
    path2LCA1, path2LCA2 = findLCA(path1, path2)
    print(f'Paths to least common ancestor between node {frame1} and {frame2}:')
    print(path2LCA1)
    print(path2LCA2)
