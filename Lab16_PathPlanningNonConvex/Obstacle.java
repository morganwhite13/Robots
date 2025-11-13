//Author Morgan White SN: 101184799 & Coauthor Jake Taylor SN: 

import java.util.ArrayList;
import java.awt.Point;
import java.awt.Polygon;

public class Obstacle {
  private ArrayList<Point>   vertices;   // An obstacle is just its vertices as points stored in CCW order
  private Polygon            asPolygon;  // A polygonal representation of the obstacle for display.
  private boolean            polygonWasComputed;  // true if a polygonal version of this obstacle was computed
  
  public Obstacle() {
    vertices = new ArrayList<Point>();
    polygonWasComputed = false;
  }
  
  public ArrayList<Point> getVertices() { return vertices; }
  

  public void addVertex(int x, int y) {
    vertices.add(new Point(x,y));
    polygonWasComputed = false;
    computePolygon();
  }
  
  public Point getVertex(int i) {
    return vertices.get(i);
  }
  
  public int size() { return vertices.size(); }
  
  // Return a polygonal version of the obstacle
  public Polygon asPolygon() {
    if (polygonWasComputed)
      return asPolygon;
    computePolygon();
    return asPolygon;
  }
  
  // Compute and store a polygon version of this obstacle, which makes things easier 
  // for displaying and for checking point inclusion
  public void computePolygon() {
    if (polygonWasComputed)
      return;
    asPolygon = new Polygon();
    for (Point v: vertices)
      asPolygon.addPoint(v.x, v.y);
    polygonWasComputed = true;
  }
  
  // Return true if the given point lies on or within the obstacle boundary.
  public boolean contains(Point p) {
    if (asPolygon == null)  
      return false;
    return asPolygon.contains(p);
  }
  
  // Determines whether or not the given point is on the boundary of the obstacle
  public boolean pointOnBoundary(Point p) {
    boolean onBoundary = false;
    for (int i=0; i<vertices.size(); i++) {
      Point v1 = vertices.get(i);
      Point v2 = vertices.get((i+1)%vertices.size());
      if (java.awt.geom.Line2D.Double.linesIntersect(p.x, p.y, p.x, p.y, v1.x, v1.y, v2.x, v2.y)) 
        onBoundary = true;
    }
    return onBoundary;
  }
  
  
  // Check if the obstacle is convex
  public boolean isConvex() {
  	

    // System.out.println("CONCVE");

    for (int i=0; i < vertices.size(); i++) {

      Point lastV = vertices.get((i-1+vertices.size())%vertices.size());
      Point curV = vertices.get(i);
      Point nextV = vertices.get((i+1)%vertices.size());

      // System.out.println("ddd " + (i-1+vertices.size())%vertices.size());
      // System.out.println("AAA " + ((curV.x-lastV.x)*(nextV.y-lastV.y)-(curV.y-lastV.y)*(nextV.x-lastV.x)));


      if ((curV.x-lastV.x)*(nextV.y-lastV.y)-(curV.y-lastV.y)*(nextV.x-lastV.x) <= 0) {
        // System.out.println("SSSSSSSSSS ");
        return false;
      }



    }
  	
    // Replace the line below with your own code
    return true;
    
    
    
  }
  
  
  
  // Decompose an obstacle into triangles.  Return an arraylist of Obstacles where
  // each obstacle is a triangle.
  public ArrayList<Obstacle> splitIntoTriangles() {
    ArrayList<Obstacle>  triangles = new ArrayList<Obstacle>();

    System.out.println("pslit");
    
    if (vertices.size() == 3) {

      // triangles.add(this);
      Obstacle ear2 = new Obstacle();
      ear2.addVertex(vertices.get(0).x, vertices.get(0).y);
      ear2.addVertex(vertices.get(1).x, vertices.get(1).y);
      ear2.addVertex(vertices.get(2).x, vertices.get(2).y);
      triangles.add(ear2);
      return triangles;


    }

    ArrayList<Point> vertices2 = new ArrayList<Point>();

    for (int i=0; i < vertices.size(); i++) {

      vertices2.add(new Point(vertices.get(i).x, vertices.get(i).y));



    }

    while (vertices2.size() > 3) {

      int earIndex = -1;
      Obstacle ear = new Obstacle();

      for (int i = 0; i < vertices2.size() && earIndex == -1; i++) {

        Point lastV = vertices2.get((i-1+vertices2.size())%vertices2.size());
        Point curV = vertices2.get(i);
        Point nextV = vertices2.get((i+1)%vertices2.size());

        if ((curV.x-lastV.x)*(nextV.y-lastV.y)-(curV.y-lastV.y)*(nextV.x-lastV.x) > 0) {
          
          // ear = new Obstacle(); //Jakob Taylor
          ear.addVertex(lastV.x, lastV.y);
          ear.addVertex(curV.x, curV.y);
          ear.addVertex(nextV.x, nextV.y);
          earIndex = i;

          for (int k = 0; k < vertices2.size(); k++) {
            Point curV2 = vertices2.get(k);

            // if (k != i-1 && k != i && k != i+1 && (ear.contains(curV2) || ear.pointOnBoundary(curV2))) {
            // if (k != (i-1+vertices2.size())%vertices2.size() && k != i && k != (i+1)%vertices2.size() && (ear.contains(curV2) || ear.pointOnBoundary(curV2))) {
            if (k != (i-1+vertices2.size())%vertices2.size() && k != i && k != (i+1)%vertices2.size() && k != i-1 && k != i+1 && (ear.contains(curV2) || ear.pointOnBoundary(curV2))) {
            // if (k != (i-1+vertices2.size())%vertices2.size() && k != i && k != (i+1)%vertices2.size() && k != i-1 && k != i+1 && ear.pointOnBoundary(curV2)) {

              // System.out.println("XXXXX " + k);
              earIndex = -1;


            }

          }
          // if (earIndex != -1)

        }




      }

      if (earIndex == -1) {

        // System.out.println("DDDD " + triangles.size());


        

        return triangles;



      } else {

        // vertices2.remove(o)
        // vertices2.remove((earIndex+1)%vertices.size());//maybe surrounding ones too
        // vertices2.remove((earIndex+1)%vertices.size());//maybe surrounding ones too
        // vertices2.remove((earIndex+1)%vertices.size());//maybe surrounding ones too
        vertices2.remove(earIndex);//maybe surrounding ones too
        // vertices2.remove((earIndex-1+vertices.size())%vertices.size());//maybe surrounding ones too

        triangles.add(ear);
      }

      



    }
    

    
    if (vertices2.size() == 3) {

      Obstacle ear2 = new Obstacle();
      ear2.addVertex(vertices2.get(0).x, vertices2.get(0).y);
      ear2.addVertex(vertices2.get(1).x, vertices2.get(1).y);
      ear2.addVertex(vertices2.get(2).x, vertices2.get(2).y);
      triangles.add(ear2);

    }
    
    // earIndex = i;
    
    // Replace the line below with your own code
    // triangles.add(this);
    
    
    System.out.println("AAA " + triangles.size());
    
    return triangles;
  }
}