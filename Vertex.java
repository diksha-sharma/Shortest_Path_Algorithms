/**
 * Created by dxs134530 on 3/21/2016.
 */
/*
 * Implementation of Shortest path problem 24.2-4 from CLRS book
 * Group Information: G13
 * Name : Diksha Sharma
 * Name : Sharol Clerit Pereira
*/
/**
 * Class to represent a vertex of a graph
 *
 *
 */

import java.util.*;

public class Vertex {
    public int name; // name of the vertex
    public boolean seen; // flag to check if the vertex has already been visited
    public Vertex parent; // parent of the vertex
    public int distance; // distance to the vertex from the source vertex
    public List<Edge> Adj, revAdj; // adjacency list; use LinkedList or ArrayList
    public ArrayList<Edge> paths; //Contains list of all possible paths from a vertex
                                  // to other vertices in graph
    public int degree; //Number of incoming edges from the vertex
    /**
     * Constructor for the vertex
     *
     * @param n
     *            : int - name of the vertex
     */
    Vertex(int n) {
        name = n;
        seen = false;
        parent = null;
        Adj = new ArrayList<Edge>();
        revAdj = new ArrayList<Edge>();   /* only for directed graphs */
        degree = 0;
    }

    /**
     * Method to represent a vertex by its name
     */
    public String toString() {
        return Integer.toString(name);
    }
}
