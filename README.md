# CITS2200
 
# CITS2200 - Data Structures and Algorithms Project: Graph Algorithms for Computer Networks

This project, completed as part of the CITS2200 course, focuses on implementing and analyzing graph algorithms to simulate a simple computer network. The project uses fundamental graph traversal techniques and the Edmonds-Karp algorithm to model, explore, and optimize network connectivity and data flow between devices.

## Project Overview

The goal of this project is to model a computer network as a graph and implement several algorithms to analyze its structure, check connectivity, and calculate paths and flow between devices. The vertices of the graph represent devices, and the edges represent the physical links between them. Key algorithms implemented include:
- **Breadth-First Search (BFS)** and **Depth-First Search (DFS)** for traversing the network.
- **Edmonds-Karp Algorithm** for calculating maximum flow between two devices.
- Various custom methods to assess connectivity, calculate paths, and optimize network performance.

### Key Features:
1. **Breadth-First Search (BFS)**:
   - Explores the graph level by level using a queue structure, ensuring all neighboring nodes are visited before moving to the next level. This is especially useful for finding the shortest path in an unweighted graph.

2. **Depth-First Search (DFS)**:
   - Recursively traverses the graph, exploring each path as deeply as possible before backtracking. DFS is used here to analyze all potential paths and check if the network is fully connected.

3. **Edmonds-Karp Algorithm**:
   - A variation of the Ford-Fulkerson algorithm, Edmonds-Karp uses BFS to find the maximum flow in a network from a source node to a sink node. This algorithm helps optimize data transmission across the network by identifying bottlenecks and ensuring efficient resource allocation.

### Methods Implemented:
- **allDevicesConnected(int[][] adjlist)**: 
   - Uses DFS to check if the graph is strongly connected, meaning every device can reach every other device.
   
- **numPaths(int[][] adjlist, int src, int dst)**:
   - Calculates the number of distinct paths between two devices using a recursive DFS approach.

- **closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries)**:
   - Finds the closest device in a subnet to the source device, using topological sorting and checking for subnet matches in the addresses of queried devices.

- **maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst)**:
   - Utilizes the Edmonds-Karp algorithm to find the maximum data transmission speed (or flow) between two devices.

## Time Complexity of Algorithms
1. **BFS and DFS**:
   - Both algorithms have a time complexity of O(V + E), where V is the number of vertices (devices) and E is the number of edges (links).
   
2. **Edmonds-Karp Algorithm**:
   - The time complexity is O(V * E^2), as the algorithm runs BFS repeatedly to find augmenting paths, with each iteration updating residual capacities.

## Performance and Complexity Analysis
- The project implements efficient graph traversal and flow algorithms to simulate real-world network scenarios. By using both DFS and BFS, the project explores different approaches to connectivity and pathfinding. The Edmonds-Karp algorithm optimizes network flow by identifying paths that maximize throughput, ensuring that the network can handle data efficiently even in congested scenarios.

- Each method is carefully designed to balance computational efficiency with accuracy, and the overall performance of the algorithms is well-suited for analyzing medium-sized networks.

## Conclusion

This project demonstrates the application of graph theory to solve problems in computer networks, including checking connectivity, counting paths, and optimizing data flow. By using BFS, DFS, and the Edmonds-Karp algorithm, the project provides a comprehensive toolset for simulating and analyzing network performance. These algorithms are crucial for network optimization, ensuring robust and efficient communication between devices.
