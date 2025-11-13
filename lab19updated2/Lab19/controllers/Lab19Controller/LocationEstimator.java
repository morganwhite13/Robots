//Author Morgan White SN: 101184799 & Coauthor Arup Kar SN: 101190352


import java.util.ArrayList;
import java.awt.Point;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class LocationEstimator {
  public static final int       NUM_SAMPLES = 20000;       // Number of samples to start with
  public static final double    PERCENT_TOLERANCE = 0.20;  // 20% randomness error
    
  private VectorMap           vectorMap;     // a vector version of the map
  private int[]               gridImage;     // For display purposes only
  private ArrayList<Pose>     currentEstimates;
  private int                 maxX, minX, maxY, minY;
  private int                 width, height;
    
    
  // A Location Estimator must have a map to start with
  public LocationEstimator(VectorMap vmp) {
    vectorMap = vmp;
    currentEstimates = new ArrayList<Pose>();
    calculateMinAndMax();
    width = vmp.getWidth();
    height = vmp.getHeight();
    gridImage = new int[width*height];
    System.out.println("Display Grid dimensions = ("+width+","+height+")");
  }
    
  public ArrayList<Pose> getPoseEstimates() { return currentEstimates; }
  
  // Find the minimum and maximum x and y of all obstacles
  public void calculateMinAndMax() {
    maxX = maxY = -1;
    minX = minY = 9999999;
    for (Obstacle ob: vectorMap.getObstacles()) {
      for (int i=0; i<ob.size(); i++) {
        Point p = ob.getVertex(i); 
        if (p.x > maxX) maxX = p.x;
        if (p.x < minX) minX = p.x;
        if (p.y > maxY) maxY = p.y;
        if (p.y < minY) minY = p.y;
      }
    }
    System.out.println("MIN("+minX+","+minY+") and MAX("+maxX+","+maxY+")");
  }
    
    
  // Return a pose that does not intersect any obstacles
  private boolean isValidPose(Pose p) {
    // If it is not inside an obstacle, then it is a good one
    for (Obstacle ob: vectorMap.getObstacles()) {
      Point pt = new Point(p.x, p.y);
      if (ob.contains(pt) || ob.pointOnBoundary(p.x, p.y))
        return false;
    }
    return true;
  }
    
  // Return a pose that does not intersect any obstacles
  private Pose getValidPose() {
    while(true) {
      Pose p = new Pose((int)(Math.random()*(maxX-minX) + minX), (int)(Math.random()*(maxY-minY) + minY), (int)(Math.random()*360));
      if (isValidPose(p))
        return p;
    }
  }
    
  // Reset the pose estimates
  public void resetEstimates() {
    currentEstimates.clear();
    int sampleCount = 0;
    while(sampleCount < NUM_SAMPLES) {
      Pose p = getValidPose();
      currentEstimates.add(p);
      sampleCount++;
    }
  }
   
    
  // Return the intersection point of two lines (assumes that they do indeed intersect)
  public Point intersectionPoint(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
    int d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (d == 0)
      return null;
        
    int xi = ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/d;
    int yi = ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/d;
        
    Point p = new Point(xi,yi);
    if (xi < Math.min(x1,x2) || xi > Math.max(x1,x2)) return null;
    if (xi < Math.min(x3,x4) || xi > Math.max(x3,x4)) return null;
        
    return p;
  }
        
// Re-estimate the position again based on the given sensor reading (in cm)
  public void estimateFromReading(double d) {
  
    ArrayList<Pose> goodEstimates = new ArrayList<Pose>();
    
    for (int i = 0; i < currentEstimates.size(); i++) {
    
      Pose curPose = currentEstimates.get(i);
      if (isValidPose(curPose) && isGoodEstimate(curPose, d)) {
      
      
        goodEstimates.add(curPose);
      
      
      
      }
    
    
    
    }
    
    if (goodEstimates.isEmpty()) {
    
    
      //currentEstimates = resetEstimates();
      resetEstimates();
    
    
    } else {
    
    
      double c = NUM_SAMPLES / goodEstimates.size();
      currentEstimates.clear();
      for (int i = 0; i < goodEstimates.size(); i++) {
      
        for (int j = 0; j < c; j++) {
        
          currentEstimates.add(goodEstimates.get(i));
        
        }
      
      
      }
    
    
    }
  
  
    
  }
  
  
  public boolean isGoodEstimate(Pose p2, double d) {
  
    Point pPrime = new Point(p2.x + ((int) ((1+PERCENT_TOLERANCE)*d*Math.cos(Math.toRadians(p2.angle)))), p2.y + ((int) ((1+PERCENT_TOLERANCE)*d*Math.sin(Math.toRadians(p2.angle)))));
    //Point pPrime = new Point(p2.x + ((int) ((1+PERCENT_TOLERANCE)*d*Math.cos((p2.angle)))), p2.y + ((int) ((1+PERCENT_TOLERANCE)*d*Math.sin((p2.angle)))));
    Point p = new Point(p2.x, p2.y);
  
    // System.out.println("p is " + p + " pprime is " + pPrime + " d is " + d);
  
    Point q = null;
    Point qPrime = null;
    
    
    ArrayList<Obstacle> obs = vectorMap.getObstacles();
    for (int i = 0; i < obs.size(); i++) {
    
      Obstacle obj = obs.get(i);
      ArrayList<Point> verts = obj.getVertices();      
      Point point1;
      Point point2;
      for (int j = 0; j < verts.size(); j++) {
      
        point1 = verts.get(j);
        point2 = verts.get((j + 1)%verts.size());
        // System.out.println("p is " + p + " pprime is " + pPrime + " d is " + d + "point 1 is " + point1 + "point 2 is " + point2);
        
        if (java.awt.geom.Line2D.Double.linesIntersect(p.x, p.y, pPrime.x, pPrime.y, point1.x, point1.y, point2.x, point2.y)) {
        
        
          
          qPrime = intersectionPoint(p.x, p.y, pPrime.x, pPrime.y, point1.x, point1.y, point2.x, point2.y);
          
          if (q == null && qPrime != null) {
          
            q = qPrime;
          
          
          } else if (qPrime != null && Math.abs(p.distance(qPrime)) < Math.abs(p.distance(q))) {
          
          
            q = qPrime;
          
          
          
          }
        
        
        }
      
      
      
      }
      
      
    
    
    }  
    if (q == null) {
      
      return false;
    
    
    } else if (Math.abs(p.distance(q)) > (1-PERCENT_TOLERANCE)*d && Math.abs(p.distance(q)) < (1+PERCENT_TOLERANCE)*d) {
    
      return true;
    
    
    }
    
    
    return false;
  
  
  
  
  }
    
    
  // Update all the estimates, based on the given forward movement (in pixels)
  public void updateLocation(int distance) {
  
  
    for (int i = 0; i < currentEstimates.size(); i++) {
    
      //System.out.println("test");
    
      currentEstimates.get(i).x += distance * (1 + (Math.random()*(2*PERCENT_TOLERANCE) - PERCENT_TOLERANCE)) * Math.cos(Math.toRadians(currentEstimates.get(i).angle));
      currentEstimates.get(i).y += distance * (1 + (Math.random()*(2*PERCENT_TOLERANCE) - PERCENT_TOLERANCE)) * Math.sin(Math.toRadians(currentEstimates.get(i).angle));
    
    
    }
    
  

  }
    
  // Update all the estimates, based on the given change in orientation (in degrees)
  public void updateOrientation(int degrees) {
  
    for (int i = 0; i < currentEstimates.size(); i++) {
    
      currentEstimates.get(i).angle += degrees * (1 + (Math.random()*(2*PERCENT_TOLERANCE) - PERCENT_TOLERANCE));
      
    
    
    }

  }
  
  
  // Draw the location estimates
  public void draw(Display aPen, int magnification) {
    try {
      // Erase the old grid
      for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
          gridImage[y*width + x] = 0x00000000;
        }
      }
      for (Pose p: currentEstimates) {
        if ((p.x < 0) || (p.x >= width) || (p.y < 0) || (p.y>=height))
          continue;
        gridImage[(height - 1 - p.y)*width + p.x] = 0xFFFFFFFF;
      }
      
      ImageRef imageOfGrid = aPen.imageNew(width, height, gridImage, Display.ARGB);
      aPen.imagePaste(imageOfGrid, 0, 0, false);
    } catch (java.util.ConcurrentModificationException ex) {
     // Do nothing please
    }
  }
}