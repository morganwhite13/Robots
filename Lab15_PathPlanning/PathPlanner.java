//Author Morgan White SN: 101184799 & Coauthor Katherine Archibald SN: 101178731

import java.util.ArrayList;
import java.awt.*;
//import java.awt.geom.Line2D.*;
//import java.awt.geom.*;

public class PathPlanner {
  private static final Color   OBSTACLE_COLOR = new Color(150, 200, 255); // light pale blue
  private static final Color   OBSTACLE_ID_COLOR = new Color(150, 0, 0);  // Dark red

  private Point 	    start;            // Start point of robot
  private Point	            end;              // End point of robot
  private VectorMap         vmap;             // Needs to be loaded
  private ArrayList<Point>  supportLines;     // Needs to be computed.  Support lines from start location only
  private Graph             visibilityGraph;  // Needs to be computed
  
  public static boolean     ShowObstacles = true;
  public static boolean     ShowStartDest = true;
  public static boolean     ShowSupportLines = false;
  public static boolean     ShowGraph = false;
  public static boolean     ShowObstacleLabels = false;
  
  // A Path Planner also requires a starting and end location, but these will be set later
  public PathPlanner(VectorMap  vm) {
    start = null;  // Needs to be provided later
    end = null;    // Needs to be provided later
    vmap = vm;
    supportLines = null;    //Needs to be computed
    visibilityGraph = null; // Needs to be computed
  }

  // These are used to refresh the start or end points or the map
  public void setStart(int x, int y) { start = new Point(x, y); }
  public void setEnd(int x, int y) { end = new Point(x,y); }
  public void setVectorMap(VectorMap m) { vmap = m; }
  public Graph getVisibilityGraph() { return visibilityGraph; }

  

  public ArrayList<Point> computeSupportPointsFrom(ArrayList<Obstacle> obstacles, int x, int y) {
    ArrayList<Point>      supports = new ArrayList<Point>();
    
    // WRITE YOUR CODE HERE

    for (int i = 0; i < obstacles.size(); i++) {

      Point pl = null;
      Point pr = null;

      for (int j = 0; j < obstacles.get(i).size(); j++) {

        double t1 = (obstacles.get(i).getVertex(j).x-x)*(obstacles.get(i).getVertex((j+1)%obstacles.get(i).size()).y-y) - (obstacles.get(i).getVertex(j).y-y)*(obstacles.get(i).getVertex((j+1)%obstacles.get(i).size()).x-x);
        double t2 = (obstacles.get(i).getVertex(j).x-x)*(obstacles.get(i).getVertex((i-1+obstacles.get(i).size())%obstacles.get(i).size()).y-y) - (obstacles.get(i).getVertex(j).y-y)*(obstacles.get(i).getVertex((i-1+obstacles.get(i).size())%obstacles.get(i).size()).x-x);




        if (t1 <= 0 && t2 <= 0 && pl == null) {

          pl = new Point(obstacles.get(i).getVertex(j).x, obstacles.get(i).getVertex(j).y);
          //supports.add(new Point(obstacles.get(i).getVertex(j).x, obstacles.get(i).getVertex(j).y));


        }
        if (t1 >= 0 && t2 >= 0 && pr == null) {

          pr = new Point(obstacles.get(i).getVertex(j).x, obstacles.get(i).getVertex(j).y);


          //supports.add(new Point(obstacles.get(i).getVertex(j).x, obstacles.get(i).getVertex(j).y));


        }


      }
      
      if (pl != null) {
        supports.add(pl);
      }

      if (pr != null) {
        supports.add(pr);
      }



    }

    supports.add(end);


    for (int i = 0; i < supports.size(); i++) {
      boolean stopped = false;

      for (int j = 0; j < obstacles.size() && !stopped; j++) {

        for (int u = 0; u < obstacles.get(j).size() && !stopped; u++) {


          int x2 = supports.get(i).x;
          int y2 = supports.get(i).y;
          int x3 = obstacles.get(j).getVertex(u).x;
          int y3 = obstacles.get(j).getVertex(u).y;
          int x4 = obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).x;
          int y4 = obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).y;
          



          // if (!obstacles.get(j).contains(supports.get(i)) && java.awt.geom.Line2D.Double.linesIntersect(x,y, supports.get(i).x,supports.get(i).y, obstacles.get(j).getVertex(u).x,obstacles.get(j).getVertex(u).y, obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).x,obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).y)) {

          //   supports.remove(i);
          //   i--;
          //   stopped = true;


          // }
          //if (java.awt.geom.Line2D.Double.linesIntersect(x,y, supports.get(i).x,supports.get(i).y, obstacles.get(j).getVertex(u).x,obstacles.get(j).getVertex(u).y, obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).x,obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()).y)) {
          //if (java.awt.geom.Line2D.Double.linesIntersect(x,y, x2,y2, x3,y3, x4,y4)) {
          //if (supports.get(i) != obstacles.get(j).getVertex(u) && supports.get(i) != obstacles.get(j).getVertex((u+1)%obstacles.get(j).size()) && java.awt.geom.Line2D.Double.linesIntersect(x,y, x2,y2, x3,y3, x4,y4)) {
          if (x2 == x3 && y2 == y3) {


          } else if (x2 == x4 && y2 == y4) {

          } else if (x2 == x && y2 == y) {


          } else if (x == x3 && y == y3) {


          } else if (x == x4 && y == y4) {

          } else if (java.awt.geom.Line2D.Double.linesIntersect(x,y, x2,y2, x3,y3, x4,y4)) {


            
            supports.remove(i);
            i--;
            stopped = true;



          }



        }



      }



    }


    
    return supports;
  }


  // Create and store a new visibility graph for the given vector map
  public void computeVisibilityGraph() {
     ArrayList<Obstacle> obstacles = vmap.getObstacles();  // all obstacles in environment





    // Create and store the graph for display access later
    visibilityGraph = new Graph();
    
    // WRITE YOUR CODE HERE


    visibilityGraph.addNode(new Node(start));
    visibilityGraph.addNode(new Node(end));

    for (int i = 0; i < obstacles.size(); i++) {

     

      for (int j = 0; j < obstacles.get(i).size(); j++) {

        visibilityGraph.addNode(new Node(obstacles.get(i).getVertex(j)));


      }
      



    }


    //ArrayList<Point> startedges = computeSupportPointsFrom(obstacles, start.x, start.y);

    //visibilityGraph.addEdge(visibilityGraph.getNodes().get(0), new Node(startedges.get(0)));



    for (int i = 0; i < visibilityGraph.getNodes().size(); i++) {

      //System.out.println(visibilityGraph.getNodes().size());


      ArrayList<Point> spprts = computeSupportPointsFrom(obstacles, visibilityGraph.getNodes().get(i).getLocation().x, visibilityGraph.getNodes().get(i).getLocation().y);
      System.out.println(spprts.size());
      for (int j = 0; j < spprts.size(); j++) {

        visibilityGraph.addEdge(visibilityGraph.getNodes().get(i), new Node(spprts.get(j)));


      }



    }


    






   
  }
  
  
  
  // This procedure displays the results of the PathPlanning algorithm which includes, the obstacles, support lines and visibility graph
  public void displayResults(Graphics aPen, double resolution, Point mousePosition) {
    int magnification= (int)(1/resolution);
    try {
      if (ShowObstacles) {
        // Convert obstacles to polygons, then display the polygons
        int count = 0;
        for (Obstacle ob: vmap.getObstacles()) {
          Polygon p = new Polygon();
          Point avg = new Point();
          for (int i=0; i<ob.size(); i++) {
            int x = MapperApp.MARGIN_X/2 + ob.getVertex(i).x*magnification;
            int y = MapperApp.MARGIN_Y/2 + vmap.getHeight() - ob.getVertex(i).y*magnification;
            p.addPoint(x, y);
            avg.x += x;
            avg.y += y;
          }
        
          aPen.setColor(OBSTACLE_COLOR);
          aPen.fillPolygon(p);
          aPen.setColor(Color.black);
          aPen.drawPolygon(p);
          
          // Show the obstacle number at its center
          if (ShowObstacleLabels) {
            aPen.setColor(OBSTACLE_ID_COLOR);
            aPen.drawString(""+count++, (avg.x/p.npoints)-5, (avg.y/p.npoints)+5);
          }
        }
      }
      if (ShowStartDest) {
        // Now draw the start/end points
        Node s = new Node(new Point(start.x, start.y));
        Node e = new Node(new Point(end.x, end.y));
        s.draw(aPen, vmap.getHeight(), magnification);
        e.draw(aPen, vmap.getHeight(), magnification);
      }
      if (ShowSupportLines) {
        aPen.setColor(Color.red);
        Point visPoint = new Point((int)((mousePosition.x-MapperApp.MARGIN_X/2)*resolution), (int)(vmap.getHeight()*resolution - (mousePosition.y-MapperApp.MARGIN_Y/2)*resolution));
        supportLines = computeSupportPointsFrom(vmap.getObstacles(), visPoint.x, visPoint.y);
        for (Point p: supportLines) {
          if (p != null)
            aPen.drawLine(MapperApp.MARGIN_X/2 + visPoint.x*magnification, MapperApp.MARGIN_Y/2 + vmap.getHeight() - visPoint.y*magnification,
                          MapperApp.MARGIN_X/2 + p.x*magnification, MapperApp.MARGIN_Y/2 + vmap.getHeight() - p.y*magnification);
        }
      }
      if (ShowGraph && (visibilityGraph != null)) {
        visibilityGraph.draw(aPen, vmap.getHeight(), magnification);
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}