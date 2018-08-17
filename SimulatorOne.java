/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
//package Graphs;

import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Queue;
import java.util.Map;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.Arrays;
import java.util.*;

// Used to signal violations of preconditions for
// various shortest path algorithms.
class GraphException extends RuntimeException
{
    /**
    * 
    */
   private static final long serialVersionUID = 1L;

   public GraphException( String name )
    {
        super( name );
    }
}

// Represents an edge in the graph.
class Edge
{
    public Vertex     dest;   // Second vertex in Edge
    public double     cost;   // Edge cost
    
    public Edge( Vertex d, double c )
    {
        dest = d;
        cost = c;
    }
}

// Represents an entry in the priority queue for Dijkstra's algorithm.
class Path implements Comparable<Path>
{
    public Vertex     dest;   // w
    public double     cost;   // d(w)
    
    public Path( Vertex d, double c )
    {
        dest = d;
        cost = c;
    }
    
    public int compareTo( Path rhs )
    {
        double otherCost = rhs.cost;
        
        return cost < otherCost ? -1 : cost > otherCost ? 1 : 0;
    }
}

// Represents a vertex in the graph.
class Vertex
{
    public String     name;   // Vertex name
    public List<Edge> adj;    // Adjacent vertices
    public double     dist;   // Cost
    public Vertex     prev;   // Previous vertex on shortest path
    public int        scratch;// Extra variable used in algorithm

    public Vertex( String nm )
      { name = nm; adj = new LinkedList<Edge>( ); reset( ); }

    public void reset( )
    //  { dist = Graph.INFINITY; prev = null; pos = null; scratch = 0; }    
    { dist = Graph.INFINITY; prev = null; scratch = 0; }
      
   // public PairingHeap.Position<Path> pos;  // Used for dijkstra2 (Chapter 23)
}

// Graph class: evaluate shortest paths.
//
// CONSTRUCTION: with no parameters.
//
// ******************PUBLIC OPERATIONS**********************
// void addEdge( String v, String w, double cvw )
//                              --> Add additional edge
// void printPath( String w )   --> Print path after alg is run
// void unweighted( String s )  --> Single-source unweighted
// void dijkstra( String s )    --> Single-source weighted
// void negative( String s )    --> Single-source negative weighted
// void acyclic( String s )     --> Single-source acyclic
// ******************ERRORS*********************************
// Some error checking is performed to make sure graph is ok,
// and to make sure graph satisfies properties needed by each
// algorithm.  Exceptions are thrown if errors are detected.

class Graph
{
    public static final double INFINITY = Double.MAX_VALUE;
    private Map<String,Vertex> vertexMap = new HashMap<String,Vertex>( );

    /**
     * Add a new edge to the graph.
     */
    public void addEdge( String sourceName, String destName, double cost )
    {
        Vertex v = getVertex( sourceName );
        Vertex w = getVertex( destName );
        v.adj.add( new Edge( w, cost ) );
    }
    
     public Double cost( String destName )
    {
        Vertex w = vertexMap.get( destName );
        if( w == null )
            throw new NoSuchElementException( "Destination vertex not found" );
        else if( w.dist == INFINITY )
            System.out.println( destName + " is unreachable" );
        else
        {
            //System.out.println( "(Cost is: " + w.dist + ") " );
           
           // printPath( w );
            //System.out.println( );
        }
        return w.dist;
    }

    /**
     * Driver routine to handle unreachables and print total cost.
     * It calls recursive routine to print shortest path to
     * destNode after a shortest path algorithm has run.
     */
    public String printPath( String destName )
    {
         
        Vertex w = vertexMap.get( destName );
        if( w == null )
            throw new NoSuchElementException( "Destination vertex not found" );
        else if( w.dist == INFINITY )
            System.out.println( destName + " is unreachable" );
        else
        {
            //System.out.println( "(Cost is: " + w.dist + ") " );
           
           // printPath( w );
            //System.out.println( );
        }
        return " "+ printPath( w );
    }
    
        
    /**
     * If vertexName is not present, add it to vertexMap.
     * In either case, return the Vertex.
     */
    private Vertex getVertex( String vertexName )
    {
        Vertex v = vertexMap.get( vertexName );
        if( v == null )
        {
            v = new Vertex( vertexName );
            vertexMap.put( vertexName, v );
        }
        return v;
    }

    /**
     * Recursive routine to print shortest path to dest
     * after running shortest path algorithm. The path
     * is known to exist.
     */
    private String printPath( Vertex dest )
    {
        String s = "";
        if( dest.prev != null )
        {
             s += printPath( dest.prev );
             
            //System.out.print( "  " );
        }
        return  s+" "+dest.name;
    }
    
    /**
     * Initializes the vertex output info prior to running
     * any shortest path algorithm.
     */
    private void clearAll( )
    {
        for( Vertex v : vertexMap.values( ) )
            v.reset( );
    }

    /**
     * Single-source unweighted shortest-path algorithm.
     */
    public void unweighted( String startName )
    {
        clearAll( ); 

        Vertex start = vertexMap.get( startName );
        if( start == null )
            throw new NoSuchElementException( "Start vertex not found" );

        Queue<Vertex> q = new LinkedList<Vertex>( );
        q.add( start ); start.dist = 0;

        while( !q.isEmpty( ) )
        {
            Vertex v = q.remove( );

            for( Edge e : v.adj )
            {
                Vertex w = e.dest;
                if( w.dist == INFINITY )
                {
                    w.dist = v.dist + 1;
                    w.prev = v;
                    q.add( w );
                }
            }
        }
    }

    /**
     * Single-source weighted shortest-path algorithm. (Dijkstra) 
     * using priority queues based on the binary heap
     */
    public void dijkstra( String startName )
    {
        PriorityQueue<Path> pq = new PriorityQueue<Path>( );

        Vertex start = vertexMap.get( startName );
        if( start == null )
            throw new NoSuchElementException( "Start vertex not found" );

        clearAll( );
        pq.add( new Path( start, 0 ) ); start.dist = 0;
        
        int nodesSeen = 0;
        while( !pq.isEmpty( ) && nodesSeen < vertexMap.size( ) )
        {
            Path vrec = pq.remove( );
            Vertex v = vrec.dest;
            if( v.scratch != 0 )  // already processed v
                continue;
                
            v.scratch = 1;
            nodesSeen++;

            for( Edge e : v.adj )
            {
                Vertex w = e.dest;
                double cvw = e.cost;
                
                if( cvw < 0 )
                    throw new GraphException( "Graph has negative edges" );
                    
                if( w.dist > v.dist + cvw )
                {
                    w.dist = v.dist +cvw;
                    w.prev = v;
                    pq.add( new Path( w, w.dist ) );
                }
            }
        }
    }

    /**
     * Single-source negative-weighted shortest-path algorithm.
     * Bellman-Ford Algorithm
     */
    public void negative( String startName )
    {
        clearAll( ); 

        Vertex start = vertexMap.get( startName );
        if( start == null )
            throw new NoSuchElementException( "Start vertex not found" );

        Queue<Vertex> q = new LinkedList<Vertex>( );
        q.add( start ); start.dist = 0; start.scratch++;

        while( !q.isEmpty( ) )
        {
            Vertex v = q.remove( );
            if( v.scratch++ > 2 * vertexMap.size( ) )
                throw new GraphException( "Negative cycle detected" );

            for( Edge e : v.adj )
            {
                Vertex w = e.dest;
                double cvw = e.cost;
                
                if( w.dist > v.dist + cvw )
                {
                    w.dist = v.dist + cvw;
                    w.prev = v;
                      // Enqueue only if not already on the queue
                    if( w.scratch++ % 2 == 0 )
                        q.add( w );
                    else
                        w.scratch--;  // undo the enqueue increment    
                }
            }
        }
    }

    /**
     * Single-source negative-weighted acyclic-graph shortest-path algorithm.
     */
    public void acyclic( String startName )
    {
        Vertex start = vertexMap.get( startName );
        if( start == null )
            throw new NoSuchElementException( "Start vertex not found" );

        clearAll( ); 
        Queue<Vertex> q = new LinkedList<Vertex>( );
        start.dist = 0;
        
          // Compute the indegrees
      Collection<Vertex> vertexSet = vertexMap.values( );
        for( Vertex v : vertexSet )
            for( Edge e : v.adj )
                e.dest.scratch++;
            
          // Enqueue vertices of indegree zero
        for( Vertex v : vertexSet )
            if( v.scratch == 0 )
                q.add( v );
       
        int iterations;
        for( iterations = 0; !q.isEmpty( ); iterations++ )
        {
            Vertex v = q.remove( );

            for( Edge e : v.adj )
            {
                Vertex w = e.dest;
                double cvw = e.cost;
                
                if( --w.scratch == 0 )
                    q.add( w );
                
                if( v.dist == INFINITY )
                    continue;    
                
                if( w.dist > v.dist + cvw )
                {
                    w.dist = v.dist + cvw;
                    w.prev = v;
                }
            }
        }
        
        if( iterations != vertexMap.size( ) )
            throw new GraphException( "Graph has a cycle!" );
    }

    /**
     * Process a request; return false if end of file.
     */
    public static boolean processRequest(Scanner in, Graph g, String[] lastfour )
    {
        try
        {
          //int c = 0;
          String pos = lastfour[0];
          String vpos = lastfour[2];
          String hn = lastfour[1];
          String[] harray = hn.split(" ");
          int h = Integer.parseInt(pos);
          int v = Integer.parseInt(vpos);
          String vn = lastfour[3];
          String[] varray = vn.split(" ");


            //System.out.print( "Enter algorithm (u, d, n, a ): " );
             
             for (int i=0;i<v;i++){
                  int c =0;
                  String[][] aP = new String[h][2];
                  System.out.println("victim " + varray[i]);
                  double cst = 0;
                  double ct = 0;
                  double vcst = 0;
                  double hcst= 0;
                  //double lowestcost = Graph.INFINITY;
                  String[] lowestpath = new String[1000];
                  ArrayList<Double> alcst = new ArrayList<Double>();
                  for(int j = 0; j<h; j++){
                      //from victim to the hospital
                      g.dijkstra( varray[i] );
                      String b = g.printPath( harray[j]);
                      
                      vcst = g.cost(harray[j]);
                      
                      //fron hospital to the victim
                      g.dijkstra( harray[j] );
                      String a = g.printPath( varray[i] );
                      
                      hcst = g.cost(varray[i]);
                                            
                      ct = vcst+ hcst;
                      alcst.add(ct);
                      
                  }
                   Collections.sort(alcst);
                   double lowestcost = alcst.get(0);
                  
                  for (int j=0;j<h;j++){
                     
             
                     //from victim to the hospital
                      g.dijkstra( varray[i] );
                      String b = g.printPath( harray[j]);
                      
                      vcst = g.cost(harray[j]);
                      
                      //fron hospital to the victim
                      g.dijkstra( harray[j] );
                      String a = g.printPath( varray[i] );
                      
                      hcst = g.cost(varray[i]);
                                            
                      cst = vcst+ hcst;
                      String csst = " "+cst;
                         String path;  
                      if(h > 7 && b.length() > 0)
                      path = a.substring(2)+b.substring(4);
                     else
                        path = a.substring(2)+b.substring(3);
                      
                      if(cst < lowestcost || cst == lowestcost){
                        
                     System.out.println("hospital " +harray[j]);
                     System.out.println(path);
                                                
                      }
                      //System.out.println(lowestcost);// + csst);
                      //cst = 0;
                         
                  }

                  /*String[] newP;
                   for(String p : lowestpath){
                      
                      //newP = p.split(" ");
                   if(p != null){
                     System.out.println("hospital " +p.substring(0,2));
                     System.out.println(p);}
                     }
                   lowestpath = null;*/
                 // System.out.println("hospital " + harray[i]);
                            
             }
                          
             
             
             
             

                   }
        catch( NoSuchElementException e )
          { return false; }
        catch( GraphException e )
          { System.err.println( e ); }
        return true;
    }
    /**
     * A main routine that:
     * 1. Reads a file containing edges (supplied as a command-line parameter);
     * 2. Forms the graph;
     * 3. Repeatedly prompts for two vertices and
     *    runs the shortest path algorithm.
     * The data file is a sequence of lines of the format
     *    source destination cost
     */
}
public class SimulatorOne
{
    public static void main( String [] args ) throws Exception
    {
        Graph g = new Graph( );

        Scanner s = new Scanner(System.in);
        //Scanner s = new Scanner(new FileReader("inputGraph"));
        String line = s.nextLine();
        int numOfnodes = Integer.parseInt( line );
        String source = "";
        StringTokenizer st = null;
        for(int i = 0; i< numOfnodes; i++){
            line = s.nextLine();
            st = new StringTokenizer( line );
            source  = st.nextToken( );
            while(st.hasMoreTokens()){ 
                 String dest    = st.nextToken( );
                 double    cost    = Double.parseDouble( st.nextToken( ) );       
                 g.addEdge( source, dest, cost );   
            }       
        }
          System.out.println();
        String[] lastfour = new String[4]; 
        for(int j = 0 ; j < 4 ; j++){
          line = s.nextLine();
          lastfour[j] = line ;
            
         }
         //for(String pa : lastfour)
         ///   System.out.println(pa);
         int counter = 0;
        //Scanner in =new Scanner(System.in);
        while(g.processRequest(s,g,lastfour))
            if(counter == 0)
              break;
    }
}