/**
 * Created by dxs134530 on 3/21/2016.
 */
/*
 * Implementation of Shortest path problem 24.2-4 from CLRS book
 * Group Information: G13
 * Name : Diksha Sharma
 * Name : Sharol Clerit Pereira
*/
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Scanner;

//Reference: https://github.com/gzc/CLRS/blob/master/C24-Single-Source-Shortest-Paths/24.2.md
//Reference: http://www.stolerman.net/studies/cs521/cs521_fall_2011_sol4.pdf
//Reference: https://www.youtube.com/watch?v=MgM7DSjFVkU
//Reference: https://www.dyclassroom.com/graph/detecting-negative-cycle-using-bellman-ford-algorithm
public class ShortestPaths
{
    public static Graph g;
    public static int[] distance;
    public static int[] predecessor;
    public static int[] pathCount;
    private static int phase = 0;
    public static boolean equalWeight = true;
    final public static int INF = Integer.MAX_VALUE;
    private static long startTime, endTime, elapsedTime;

    //Program Arguments: "E:\\CS6301 - Implementation of advanced data structures & algorithms\\HW\\Long Project 3\\data\\1.txt"
    public static void main(String[] args) throws FileNotFoundException
    {
        Scanner in;
        if (args.length ==  0)
        {
            in = new Scanner(System.in);
        }
        else
        {
            File fInputFile = new File(args[0]);
            in = new Scanner(fInputFile);
        }
        //Read the directed input graph
        g = Graph.readGraph(in, true);
        Vertex u = g.verts.get(1);
        Edge e = u.Adj.get(0);
        boolean bPositiveWeight = true;
        int weight = e.Weight;
        for (Vertex v : g)
        {
            for (Edge ed : v.Adj)
            {
                if (ed.Weight != weight)
                {
                    equalWeight = false;
                }
                if(ed.Weight < 0)
                {
                    bPositiveWeight = false;
                }
            }
        }

        if (equalWeight)
        {
            for (Vertex w : g)
            {
                w.distance = INF;
                w.parent = null;
                w.seen = false;
            }
            bfs();
        }
        else if (bPositiveWeight == true)
        {
            dijkstra();
        }

        initializeGraph();
        timer();
        //dag();
        //initializeGraph();
        //findShortestPaths();
        timer();
    }

    public static void dijkstra()
    {

    }

    public static ArrayList<Vertex> toplogicalOrder()
    {
        Queue<Vertex> q = new LinkedList<Vertex>();
        ArrayList<Vertex> order = new ArrayList<Vertex>();
        Vertex v;
        // Invariant : u is the next node of the graph to be processed
        // q is the queue storing nodes with degree 0
        for (Vertex u : g)
        {
            if (u.degree == 0)
                q.add(u);
        }
        // Invariant : u is the next node of the graph to be processed
        // q is the queue storing nodes with degree 0
        // e is the edge of the node u being processed
        // v is the node adjacent to u

        while (!q.isEmpty())
        {
            Vertex u = q.remove();
            order.add(u);
            for (Edge e : u.Adj)
            {
                v = e.otherEnd(u);
                v.degree--;
                if (v.degree == 0)
                    q.add(v);
            }
        }
        // Invariant : p is node in ArrayList order
        for (Vertex p : order)
        {
            if (p.degree > 0)
            { // If any node has degree then it is a cycle
                return null;
            }
        }
        return order;
    }

    public static void dag()
    {
        Vertex start;
        Vertex end;
        boolean bRelaxed = false;
        ArrayList<Vertex> verticesOrder = toplogicalOrder();
        //Do the process for all vertices in the graph
        while(!verticesOrder.isEmpty())
        {
            start = verticesOrder.get(0);
            verticesOrder.remove(0);
            //For each vertex in the adj list of current vertex - update the distance
            for(Edge e: start.Adj)
            {
                end = e.otherEnd(start);
                bRelaxed = relaxEdge(e, start, end);
            }
        }
        int totalDistance = 0;
        for (int i=0; i< distance.length; i++)
        {
            if(distance[i]!=INF)
            {
                totalDistance += distance[i];
            }
        }
        System.out.println();
        System.out.println("DAG "+ totalDistance);
        printDAGOutput();
    }

    //This method prints the final output
    public static void printDAGOutput()
    {
        //System.out.println();
        for(int i=1; i< distance.length; i++)
        {
            if(i==1)
            {
                if(distance[i] != INF)
                {
                    System.out.println(i + "   " + distance[i] + "  1");
                }
                else
                {
                    System.out.println(i + "   INF  -");
                }

            }
            else
            {
                if(distance[i] != INF)
                {
                    System.out.println(i + "   " + distance[i] + "  " + predecessor[i]);
                }
                else
                {
                    System.out.println(i + "   INF  -");
                }
            }
        }
    }

    public static boolean relaxEdge(Edge e, Vertex u, Vertex v)
    {
        //u is the start vertex
        //v is the end vertex
        if(distance[v.name] > e.Weight + distance[u.name] && distance[u.name] != INF)
        {
            distance[v.name] = e.Weight + distance[u.name];
            predecessor[v.name] = u.name;
            return true;
        }
        else
        {
            return false;
        }
    }

    //This method sets distance of each vertex to other vertices as Infinity and
    //the predecessor of each vertex to 0 (value of 0 means no vertex is set as predecessor)
    public static void initializeGraph()
    {
        distance = new int[g.verts.size()]; //There is no vertex with name = 0 - so we don't use index 0
        predecessor = new int[g.verts.size()];
        pathCount = new int[g.verts.size()];
        for(int i=1; i< distance.length; i++)
        {
            distance[i] = INF;
            predecessor[i] = 0;
            pathCount[i] = 0;
        }
        distance[1] = 0; //Distance of source vertex to itself is 0
    }

    //Finds the shortest paths from the source vertex to all other vertices in the graph
    public static void findShortestPaths()
    {
        //Initialize vertex 1 as source vertex
        Vertex source = g.verts.get(1);
        Vertex start = source;
        Vertex end;
        boolean bLastIterationChangeInDistance = false;

        //We would do n iterations of this and relax the edges in each of the iterations
        //In last iteration we would check for negative cycle. If there is any change in
        //distance array in last iteration - we have found a negative cycle.
        boolean bRelaxed = false;
        for(int iIteration = 1; iIteration < g.verts.size(); iIteration++)
        {
            //Do the process for all vertices in the graph
            for(int vIndex = 1; vIndex < g.verts.size(); vIndex++)
            {
                start = g.verts.get(vIndex);
                //For each vertex in the adj list of current vertex - update the distance
                for(Edge e: start.Adj)
                {
                    bRelaxed = false;
                    end = e.otherEnd(start);
                    bRelaxed = relaxEdge(e, start, end);
                    /*if(distance[end.name] > e.Weight + distance[start.name])
                    {
                        distance[end.name] = e.Weight + distance[start.name];
                        predecessor[end.name] = start.name;
                        if(iIteration == g.verts.size()-1) //Last iteration check for negative cycle
                        {
                            bLastIterationChangeInDistance = true;
                            break;
                        }
                    }*/
                    if(iIteration == g.verts.size()-1 && bRelaxed == true) //Last iteration check for negative cycle
                    {
                        bLastIterationChangeInDistance = true;
                        break;
                    }
                }

                if(bLastIterationChangeInDistance == true)
                {
                    System.out.println();
                    System.out.println("Non-positive cycle in graph. DAC is not applicable.");
                    findNegativeCycle();
                    break;
                }
            }
        }

        //Finds the number of shortest paths from source vertex to all other vertices
        if(bLastIterationChangeInDistance == false)
        {
            countShortestPaths();
        }
    }

    //This method counts the number of shortest paths from source vertex
    //to all other vertices in graph using the edges selected in shortest path sub graph
    public static void countShortestPaths()
    {
        findSubGraphEdges();
        Vertex start;
        Vertex end;
        long lWeight = 1; //Counting path from vertex 1 to itself as one path
        ArrayList<Vertex> vertices = new ArrayList<Vertex>();
        vertices.add(g.verts.get(1));
        Vertex processVertex;
        Vertex next;
        //Process the vertices in levels - the total count of paths in a graph
        //if the product of all the paths on each level
        while(!vertices.isEmpty())
        {
            processVertex = vertices.get(0);
            vertices.remove(0);
            for(Edge e: processVertex.Adj)
            {
                if(e.bShortestPath == true)
                {
                    next = e.otherEnd(processVertex);

                    vertices.add(next);
                    pathCount[next.name]++;
                }
            }
        }
        //Print the final output
        System.out.println();
        for(int i=1; i< pathCount.length; i++)
        {
            lWeight= lWeight + pathCount[i];
        }
        System.out.println(lWeight);
        printOutput();
    }

    //Find the edges part of the shortest path sub graph and sets their "bShortestPath" flags to true
    public static void findSubGraphEdges()
    {
        Vertex source = g.verts.get(1);
        Vertex start = source;
        Vertex end;
        for(int i=1; i< g.verts.size(); i++)
        {
            start = g.verts.get(i);
            //For each vertex in the adj list of current vertex - update the distance
            for(Edge e: start.Adj)
            {
                end = e.otherEnd(start);
                if(distance[end.name] == distance[start.name] + e.Weight)
                {
                    e.bShortestPath = true;
                }
            }
        }
    }

    //This method prints the final output
    public static void printOutput()
    {
        System.out.println();
        for(int i=1; i< distance.length; i++)
        {
            if(i==1)
            {
                if(distance[i] > Integer.MIN_VALUE && distance[i] < Integer.MAX_VALUE)
                {
                    System.out.println(i + "   " + distance[i] + "  1");
                }
                else
                {
                    System.out.println(i + "   INF  1");
                }

            }
            else
            {
                if(distance[i] > Integer.MIN_VALUE && distance[i] < Integer.MAX_VALUE)
                {
                    System.out.println(i + "   " + distance[i] + "  " + pathCount[i]);
                }
                else
                {
                    System.out.println(i + "   INF  " + pathCount[i]);
                }
            }
        }
    }

    //Finds the negative weight cycle in the graph and prints it on system console
    public static void findNegativeCycle()
    {

    }

    //Finds the zero weight cycle in the graph and prints it on system console
    public static void findZeroCycle()
    {
        Vertex start = g.verts.get(1);
        Vertex end;
        ArrayList<Edge> zeroCycle = new ArrayList<Edge>();
        ArrayList<Vertex> zeroVertices = new ArrayList<Vertex>();

        for(Edge e: start.Adj)
        {
            if(e.bShortestPath == true && e.Weight == 0)
            {
                zeroCycle.add(e);
                end = e.otherEnd(start);
                if(!zeroVertices.contains(start))
                {
                    zeroVertices.add(start);
                }
                else
                {
                    //Cycle found

                }

                if(!zeroVertices.contains(end))
                {
                    zeroVertices.add(end);
                }
            }
        }
    }

    //This method prints the distance and predecessor arrays
    public static void printDistanceAndPredecessorArrays()
    {
        System.out.println("Distance array ");
        for(int i=1; i< distance.length-1; i++)
        {
            System.out.print(distance[i] + ", ");
        }
        System.out.print(distance[distance.length-1]);
        System.out.println();
        System.out.println("Predecessor array ");
        for(int i=1; i< predecessor.length-1; i++)
        {
            System.out.print(predecessor[i] + ", ");
        }
        System.out.print(predecessor[distance.length-1]);
    }

    public static void bfs()
    {
        Queue<Vertex> q = new LinkedList<Vertex>();
        Vertex src = g.verts.get(1);
        q.add(src);
        int distance = 0;

        src.distance = 0;
        src.seen = true;

        while (!q.isEmpty())
        {
            Vertex u = q.remove();
            for (Edge e : u.Adj)
            {
                Vertex v = e.otherEnd(u);
                if (!v.seen)
                {
                    v.distance = u.distance + 1;
                    //System.out.println("u distc is " + v.name + " "+v.distance);
                    v.parent = u;
                    q.add(v);
                    v.seen = true;
                }
            }
        }
        for (Vertex w : g)
        {
            if(w.distance!=INF)
            {
                distance += w.distance;
            }
            //	System.out.println(w.name + " "+distance);
        }
        System.out.println("BFS "+ distance);
    }

    public static void timer()
    {
        if (phase == 0)
        {
            startTime = System.currentTimeMillis();
            phase = 1;
        }
        else
        {
            endTime = System.currentTimeMillis();
            elapsedTime = endTime - startTime;
            System.out.println("Time: " + elapsedTime + " msec.");
            memory();
            phase = 0;
        }
    }

    public static void memory()
    {
        long memAvailable = Runtime.getRuntime().totalMemory();
        long memUsed = memAvailable - Runtime.getRuntime().freeMemory();
        System.out.println("Memory: " + memUsed / 1000000 + " MB / " + memAvailable / 1000000 + " MB.");
    }

}



