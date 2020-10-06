ASSIGNMENT 03:

Code submitted by Huzefa Kagalwala (C48290423) and Rahil Modi (C14109603)

1. astar.py is the code to make the start node reach the goal node using the A* Algorithm. 
RESULTS:

cost = [1, 1, 10]
['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

cost = [10, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

cost = [1, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

2. dijkstra.py is the code to make the start node go to the goal node using the Dijkstra's Algorithm. The only difference here is that there will be no heuristic to guide the search, so, the heuristic array was made null and the code was run. As expected, for uniform costs, without any heuristic to guide it, Dijkstra evaluated more nodes to reach the goal. If penalties are given, then the same results are obtained as A*.
RESULTS:
cost = [1, 1, 10]:
['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

cost = [10, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(1, 3, 0)
(2, 2, 1)
(0, 3, 0)
(2, 1, 1)
(2, 0, 1)

As we can see here, more nodes were opened

cost = [1, 1, 1]:
['-', '-', '-', '-', '-', '-']
['-', '-', '-', '-', '-', '-']
['*', 'F', 'F', 'L', '-', '-']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

Expanded Nodes
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 2, 1)
(2, 5, 3)
(0, 3, 0)
(2, 1, 1)
(1, 5, 0)
(0, 4, 3)
(2, 0, 1)

Here too more nodes were opened.
Hence, A* is more efficient than Dijkstra