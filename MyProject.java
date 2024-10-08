/**
 * CITS2200 Project
 * Name: Vlad Gavrikov, Nicodemus Ong
 * Student ID: 22471649, 22607943
 */

import java.util.*;

public class MyProject implements Project {

    /**
     * Implementation of allDevicesConnected
     * @param adjlist The adjacency list describing the links between devices
     * @return true if the graph is strongly connected, else return false
     */

    public boolean allDevicesConnected(int[][] adjlist) {
        // to keep track of whether a vertex is visited or not
        boolean[] visited = new boolean[adjlist.length];
        int[][] adjlistINV = new int[adjlist.length][adjlist.length];
        for (int[] row: adjlistINV)
            Arrays.fill(row, -1);
        // choose a starting point
        int v = 0;
        int[] neighborCounter = new int[adjlist.length];
        Arrays.fill(neighborCounter, 0);
        // run DFS with the starting point at V
        DFSForward(adjlist,adjlistINV,neighborCounter,v,visited);
        // If DFS Traversal does not visit all vertices, the graph is not strongly connected
        for(boolean b : visited) {
            if(!b) {
                return false;
            }
        }
        // reset visited array

        Arrays.fill(visited,false);

        v = 0;
        // run dfs again with the reverse graph
        DFSInverse(adjlistINV,v,visited);

        // If dfs does not visit all vertices, then the graph is not strongly connected
        for (boolean b : visited) {
            if(!b) {
                return false;
            }
        }
        return true; // return true when it passes both dfs for the original graph and the reverse graph
    }

    /**
     * Performs standard Depth First Search on the edge list
     * @param adjlist The adjacency list describing the links between devices
     * @param adjlistINV The inverse adjacency list describing the inverse of the links between devices
     * @param neighborCounter is the counter for the total neighbors of the vertex, it is primarily used to inverse the adjacency list
     * @param vertex the vertex parsed through the method
     * @param visited the array of visited devices
     */

    // standard DFS
    private void DFSForward(int[][] adjlist,int[][] adjlistINV,int[] neighborCounter, int vertex, boolean[] visited) {
        visited[vertex] = true;
        for (int neighbor : adjlist[vertex]) {
            adjlistINV[neighbor][neighborCounter[neighbor]] = vertex;
            neighborCounter[neighbor]++;
            if(!visited[neighbor]) {

                DFSForward(adjlist,adjlistINV,neighborCounter,neighbor,visited);
            }
        }
    }

    /**
     * Performs standard Depth First Search on the inverse edge list
     * @param adjlistINV The inverse adjacency list describing the inverse of the links between devices
     * @param vertex the vertex parsed through the method
     * @param visited the array of visited devices
     */
    private void DFSInverse(int[][] adjlistINV, int vertex, boolean[] visited) {
        visited[vertex] = true;
        for (int neighbor : adjlistINV[vertex]) {
            if(neighbor==-1) break;
            if(!visited[neighbor]) {
                DFSInverse(adjlistINV,neighbor,visited);
            }
        }
    }

    /**
     * Calculate the total number of paths from the source to the destination device
     * @param adjlist The adjacency list describing the links between devices
     * @param src The source (transmitting) device
     * @param dst The destination (receiving) device
     * @return the total number of possible paths from the source to the destination
     */

    public int numPaths(int[][] adjlist, int src, int dst) {
        boolean[] isVisited = new boolean[adjlist.length];
        ArrayList<Integer> pathList = new ArrayList<>();
        pathList.add(src);
        int counter = 0;
        counter = dfs(src,dst,isVisited,pathList,adjlist,counter);
        return counter;
    }

    /**
     * standard Depth First Search to traverse the graph, it builds a path list to ensure that it accounts for all possible paths
     * @param src The source (transmitting) device
     * @param dst The destination (receiving) device
     * @param isVisited an array to reflect if a vertex has been visited or not
     * @param localPathList an array that stores the possible paths
     * @param adjlist The adjacency list describing the links between devices
     * @param counter the total number of paths
     * @return
     */
    public int dfs(Integer src, Integer dst, boolean[] isVisited, List<Integer> localPathList,int[][] adjlist,int counter) {
        if(src.equals(dst)) {
            counter++;
            return counter;
        }
        else {
            isVisited[src] = true;
            for(Integer i : adjlist[src]) {
                if (!isVisited[i]) {
                    localPathList.add(i);
                    counter = dfs(i, dst, isVisited, localPathList,adjlist,counter);
                    localPathList.remove(i);
                }
            }
            isVisited[src] = false; // Reset the source vertex to false for DFS to recurse
        }
        return counter;
    }

    /**
     * Gets the total distance for each query from the source device
     * @param adjlist The adjacency list describing the links between devices
     * @param addrs The list of IP addresses for each device
     * @param src The source device
     * @param queries The queries to respond to for each subnet
     * @return an array storing all the distances from each query
     */

    public int[] closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries) {
        int[] arrayOfAnswers = new int[queries.length];
        Arrays.fill(arrayOfAnswers, Integer.MAX_VALUE);

        Node sourceNode = new Node(addrs[src]); // Creates a node that stores the IP address of the source device
        for(int i= 0; i<queries.length; i++) {
            if(sourceNode.checkQuerie(queries[i])) { // Checks if the source node matches the subnet of the query, if so set distance to 0
                arrayOfAnswers[i] = 0;
            }
            else{
                int[] distance = new int[adjlist.length];
                boolean[] visited = new boolean[adjlist.length];
                Arrays.fill(visited, false);
                Arrays.fill(distance, Integer.MAX_VALUE);
                Stack<Integer> stack = new Stack<Integer>();
                for (int k = 0; k< adjlist.length; k++)
                    if (!visited[k])
                        topSort(k, visited, stack, adjlist); // Topologically sort all the devices and stores them in a stack to always find shortest path
                distance[src] = 0;
                while(!stack.empty()) {
                    int u = stack.pop();
                    Node deletedNode = new Node(addrs[u]); // Creates a node object for the deleted device that stores its IP Address
                    if(deletedNode.checkQuerie(queries[i])) { // Checks if the deleted node matches the subnet of the query, if yes stores the answer
                        arrayOfAnswers[i] = distance[u];
                        break;
                    }
                    if (distance[u] != Integer.MAX_VALUE) // Checks to see the device is reachable
                    {
                        for(int z = 0; z< adjlist[u].length; z++)
                        {
                            int child = adjlist[u][z];
                            if (distance[child] > distance[u]+1) // Checks if distance of child is greater than distance already stored + 1
                                distance[child] = distance[u]+1; // If true update the distance
                        }
                    }
                }
            }

        }
        return arrayOfAnswers;
    }

    /**
     * Node class creates a node object that stores each vertex with their respective IP Addresses
     */

    public static class Node{
        private short[] ipAddress;
        // initializes each subnet to -1 as it checks for partial subnets parsed E.g 192.168 = (192.168.-1.-1)
        private short first = -1;
        private short second = -1;
        private short third = -1;
        private short forth = -1;

        /**
         * constructor for the node
         * @param ip is the IP Address parsed
         */

        public Node(short[] ip) {
            ipAddress = ip;
            if(ip.length >= 1) {
                first = ip[0];
            }
            if(ip.length >= 2) {
                second = ip[1];
            }
            if(ip.length >= 3) {
                third = ip[2];
            }
            if(ip.length == 4) {
                forth = ip[3];
            }
        }

        /**
         * Checks if the IP Address of the device matches the parsed queried IP Address
         * @param querie is the address from the query array
         * @return true if it matches, else returns false
         */

        public boolean checkQuerie(short[] querie) {
            int lengthOfQuery = querie.length;
            if(lengthOfQuery == 1) { return querie[0]==first; }
            if(lengthOfQuery == 2) { return querie[0]==first&& querie[1]==second; }
            if(lengthOfQuery == 3) { return querie[0]==first&& querie[1]==second && querie[2]==third; }
            if(lengthOfQuery == 4) { return querie[0]==first&& querie[1]==second && querie[2]==third && querie[3]==forth; }
            return true;
        }
    }

    /**
     * Sorts and stores the vertices into a stack
     * @param v is the vertex parsed
     * @param visited is the array of visited devices
     * @param stack is the stack that holds all the vertices
     * @param adjlist the adjacency list describing the links between devices
     */

    void topSort(int v, boolean[] visited, Stack<Integer> stack, int[][] adjlist) {
        visited[v] = true;
        for(int i = 0; i < adjlist[v].length; i++){
            int child = adjlist[v][i];
            if (!visited[child])
                topSort(child, visited, stack,adjlist);
        }
        stack.push(v);
    }

    /**
     * Calculates the maximum flow of the graph from the source vertex to the sink
     * @param adjlist The adjacency list describing the connections between devices
     * @param speeds The list of query row segments
     * @param src The source (transmitting) device
     * @param dst The destination (receiving) device
     * @return the maximum flow of the graph
     */

    public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {

        if(src == dst) { // Special case if the src is equals to dst, return -1
            return -1;
        }

        int vertex;
        int neighbor;
        int[] parent = new int[adjlist.length];
        int max_flow = 0;
        int[][] residualMatrix = convert(adjlist,speeds); // Create a residual graph using the convert() function

        while(bfs(residualMatrix,src,dst,parent)) { // While there is an augmenting path
            int path_flow = Integer.MAX_VALUE;
            for (neighbor = dst; neighbor != src; neighbor = parent[neighbor]) {
                vertex = parent[neighbor];
                path_flow = Math.min(path_flow, residualMatrix[vertex][neighbor]); // Gets the minimum capacity of the links from the path provided by BFS
            }

            for (neighbor = dst; neighbor != src; neighbor = parent[neighbor]) {
                vertex = parent[neighbor];
                // Update the remaining capacity of the link and reverses the edges flow
                residualMatrix[vertex][neighbor] = residualMatrix[vertex][neighbor] - path_flow;
                residualMatrix[neighbor][vertex] = residualMatrix[neighbor][vertex] + path_flow;
            }
            max_flow = max_flow + path_flow; // accumulate the max flow by adding the current path flow to it
        }
        return max_flow; // return the max flow
    }

    /**
     * performs BFS to get the augmented path for edmonds-karp's algorithm
     * @param matrix the residual matrix of the graph
     * @param src is the source vertex
     * @param dst is the sink vertex
     * @param parent is the parent array that stores the parent for each vertex
     * @return true of there is a path, else return false
     */

    private boolean bfs(int[][] matrix, int src, int dst,int[]parent) {
        // Create a visited array and mark all vertices as not visited
        boolean[] visited = new boolean[matrix.length];
        Arrays.fill(visited,false);
        Queue<Integer> q = new LinkedList<Integer>();
        q.add(src);
        visited[src] = true;
        parent[src] = -1;
        while (!q.isEmpty()) {
            int deleted = q.remove();
            for(int i = 0; i < matrix.length; i++) {
                if(!visited[i] && matrix[deleted][i] > 0) {
                    if(i == dst) {
                        parent[i] = deleted;
                        return true;
                    }
                    q.add(i);
                    parent[i] = deleted;
                    visited[i] = true;
                }
            }
        }
        return false;
    }

    /**
     * Converts the adjacency list into a residual graph
     * @param adjList the adjacency list describing the links between devices
     * @param speeds the list of query row segments
     * @return the residual matrix of the graph
     */

    public int[][] convert(int[][]adjList,int[][] speeds) {
        // Creates an empty matrix
        int[][] speedMatrix = new int[adjList.length][adjList.length];
        // Iterate through the matrix and replace 0 with 1 if there is an edge
        for (int row = 0; row < adjList.length; row++) {
            for(int column : adjList[row]) {
                speedMatrix[row][column] = 1;
            }
        }
        // Replaces the edges with its weights
        for(int i = 0; i<speedMatrix.length;i++)
        {
            for(int j = 0; j<speedMatrix[i].length;j++)
            {
                if(speedMatrix[i][j]==1) {
                    for(int x = 0; x<adjList[i].length;x++) { // This for-loop make sures that the devices in the different order in the adjacency matrix can be processed correctly
                        if(adjList[i][x]==j) {
                            speedMatrix[i][j] = speeds[i][x];
                        }
                    }
                }
            }
        }
        return speedMatrix;
    }
}

