import java.util.Collections;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.awt.*;

public class AreaCoverage {
  public static final float  ROBOT_RADIUS = 6.8f;              // in cm
  public static final float  ROBOT_DIAMETER = ROBOT_RADIUS*2;  // in cm

  private Point 	        start;            // Start point of robot
  private Point	            end;              // End point of robot
  private VectorMap         vmap;             // Needs to be loaded
  private Graph             gridGraph;        // Needs to be computed
  private ArrayList<Point>  path;             // Path in the spanning tree
  private int               edgeCount;        // the number of new edges traversed on the path
  private Node              startNode;        // starting Node of the Spanning Tree
  
  // An Area Coverage will allow the robot to perform area coverage
  public AreaCoverage(VectorMap  vm) {
    start = null;  // Needs to be provided later
    end = null;    // Needs to be provided later
    vmap = vm;
    gridGraph = null;    // Needs to be computed
    path = new ArrayList<Point>();
    startNode = null;
  }

  // These are used to refresh the start or end points or the map
  public void setStart(int x, int y) { start = new Point(x, y); }
  public void setEnd(int x, int y) { end = new Point(x,y); }
  
  public VectorMap getVectorMap() { return vmap; }
  public void setVectorMap(VectorMap m) { vmap = m; }
  public Graph getGridGraph() { return gridGraph; }


  // Create and store a new grid graph for the set of convex obstacles
  public void computeGridGraph() {
    ArrayList<Obstacle> obstacles = vmap.getObstacles();   // environment obstacles
    double width = vmap.getWidth()*vmap.getResolution();   // environment width in cm
    double height = vmap.getHeight()*vmap.getResolution(); // environment height in cm
    
    // Create and store the graph for display access later
    gridGraph = new Graph();

    // ADD YOUR CODE HERE
    double numRows = width / ROBOT_DIAMETER;
    double numCols = height / ROBOT_DIAMETER;

    Node[][] nodes = new Node[(int)numRows][(int)numCols];
    for (int i = 0; i < (int)numRows; i++) {

      for (int j = 0; j < (int)numCols; j++) {

        nodes[i][j] = new Node(new Point(((int)(i*ROBOT_DIAMETER+ROBOT_RADIUS)), ((int)(j*ROBOT_DIAMETER+ROBOT_RADIUS))));
        gridGraph.addNode(nodes[i][j]);



      }


    }

    for (int i = 0; i < (int)numRows; i++) {

      for (int j = 0; j < (int)numCols - 1; j++) {

        gridGraph.addEdge(nodes[i][j], nodes[i][j+1]);


      }


    }

    for (int i = 0; i < (int)numRows - 1; i++) {

      for (int j = 0; j < (int)numCols; j++) {

        gridGraph.addEdge(nodes[i][j], nodes[i+1][j]);


      }


    }



    //REMOVE STUFF
    ArrayList<Node> tmp = gridGraph.getNodes();
    ArrayList<Node> invalids = new ArrayList<>();
    for (int i = 0; i < tmp.size(); i++) {

      for (int j = 0; j < obstacles.size(); j++) {

        if (obstacles.get(j).contains(tmp.get(i).getLocation()) || obstacles.get(j).pointOnBoundary(tmp.get(i).getLocation())) {

          invalids.add(tmp.get(i));
          

        } else {

          ArrayList<Point> verts = obstacles.get(j).getVertices();
          for (int u = 0; u < verts.size(); u++) {

            double dst = java.awt.geom.Line2D.ptSegDist(verts.get(u).x, verts.get(u).y, verts.get((u+1)%verts.size()).x, verts.get((u+1)%verts.size()).y, tmp.get(i).getLocation().x, tmp.get(i).getLocation().y);

            if (dst <= ROBOT_RADIUS) {
              invalids.add(tmp.get(i));
            }

          }

        }


      }



    }

    for (int i = 0; i < invalids.size(); i++) {

      gridGraph.deleteNode(invalids.get(i));


    }



  }


  // Recursively compute the path in the spanning tree from the starting location having come in from edge e
  private void computeCoveragePathFrom(Node n, Edge e) {

	// ADD YOUR CODE HERE FOR PART 6
	
  }

  // Recursively compute the heights of ther nodes in the spanning tree from the starting root node
  private void computeNodeHeightsFrom(Node n) {
    n.setPrevious(n); // mark as visited
    
    ArrayList<Edge> edges = n.incidentEdges();
    if (edges.size() == 1)
      n.setDistance(0);
    else 
      n.setDistance(1);
      
    int max = 0;
    for (Edge e: edges) {
      Node n2 = e.otherEndFrom(n);
      if (n2.getPrevious() == null) {
        computeNodeHeightsFrom(n2);
        if (n2.getDistance() > max)
          max = (int)n2.getDistance();
      }
    }
    n.setDistance(n.getDistance() + max);
  }

  
  // Recursively compute the depth-first order spanning tree from the grid graph starting at node n, coming from edge e
  private void computeSpanningTreeFrom(Node n, Edge e) {
    
    // ADD YOUR CODE HERE FOR PART 5

    if (n.isSelected()) {
      return;//IDK MAYBE
    }

    if (e != null) {

      n.setSelected(true);
      e.setSelected(true);//IDK

    }

    ArrayList<Edge> edgs = n.incidentEdges();
    for (int i = 0; i < edgs.size(); i++) {

      Node otherNode;
      if (edgs.get(i).getStartNode() == n) {
        otherNode = edgs.get(i).getEndNode();
      } else {
        otherNode = edgs.get(i).getStartNode();
      }

      computeSpanningTreeFrom(otherNode, edgs.get(i));

      
    }

    
  }


  // Compute the depth-first order spanning tree from the grid graph with the starting node closest to the start point
  public void computeSpanningTree() {    
    // If there are no nodes, then stop now
    if ((gridGraph == null) || (gridGraph.getNodes().isEmpty()))
      return;
      
    // Reset all the previous pointers
    for (Node n: gridGraph.getNodes()) {
      n.setPrevious(null);
      n.setDistance(0);
    }
      
    // Find the Node closest to (x,y)
    startNode = gridGraph.getNodes().get(0);
    for (Node n: gridGraph.getNodes()) {
      if (n.getLocation().distanceSq(start.x, start.y) < startNode.getLocation().distanceSq(start.x, start.y))
        startNode = n;
    }
    
    // Recursively compute the spanning tree
    computeSpanningTreeFrom(startNode, null);
   
    // Now remove all edges that are not selected
    ArrayList<Edge>  allEdges = gridGraph.getEdges();
    for (Edge e: allEdges)
      if (!e.isSelected())
        gridGraph.deleteEdge(e);
        
    // Now compute the node heights
    for (Node n: gridGraph.getNodes()) { 
      n.setPrevious(null);               // Reset all the previous pointers
      n.setDistance(0);                  // Reset all the distances to 0
    }
    computeNodeHeightsFrom(startNode);
    
    // Now compute the coverage path
    path.clear();                        // Clear the previously-computed path, if there was one
    edgeCount = gridGraph.getEdges().size();
    for (Node n: gridGraph.getNodes())  // Reset all the previous pointers
      n.setPrevious(null);
    computeCoveragePathFrom(startNode, null);

  }


  // Get the covergae path as a set of points
  public ArrayList<Point> getCoveragePath() {
    return path;
  }
}