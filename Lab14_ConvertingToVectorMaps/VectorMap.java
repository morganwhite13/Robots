//Author Morgan White SN: 101184799 & Coauthor: Jacob Harper SN: 101111173

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;

public class VectorMap {

  public static int         LINE_TOLERANCE;
  public static boolean     ShowVector = false;

  public static final int   VERTEX_RADIUS = 3;
  public static final int   VERTEX_DIAMETER = VERTEX_RADIUS*2;
  
  private static Color  VertexColor = Color.red;
  private static Color  EdgeColor = Color.blue;
  
  private int                   width;
  private int                   height;
  private double                resolution;
  private ArrayList<Obstacle>   obstacles;
  private boolean               reduced;    // true only if the map has been reduced by applying the line-fitting

  // Create a vector map with the given with, height,  resolution
  public VectorMap(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    reduced = true;
    
    obstacles = new ArrayList<Obstacle>();
  }

  // Add an obstacle to the map
  public void addObstacle(Obstacle ob) {
    obstacles.add(ob);
  }
  
  // Clear the obstacles from the map to get reqady for a new trace
  public void clear() {
    obstacles.clear();
    reduced = false;
  }

  // Return the distance of point i to line ab
  private double distanceToLine(Point a, Point b, Point i) {
    return Math.abs((b.x - a.x)*(a.y-i.y) - (a.x-i.x)*(b.y-a.y)) / Math.sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  }
  
  // Apply the line tolerance to the obstacles so that there are a lot less vertices
  public void applyLineTolerance() {
    ArrayList<Obstacle>    newObstacles = new ArrayList<Obstacle>();
        
        
    // ADD YOUR CODE HERE

    for (int i = 0; i < obstacles.size(); i++) {

      if (obstacles.get(i).size() >= 3) {

        newObstacles.add(new Obstacle());

        int a = 0;
        Point pa = obstacles.get(i).getVertex(a);
        Point lA = null;
        for (int j = 1; j < obstacles.get(i).size(); j++) {

          pa = obstacles.get(i).getVertex(a);

          for (int u = a+1; u < j-1; u++) {

            //if (distanceToLine(pa, obstacles.get(i).getVertex(j), obstacles.get(i).getVertex(u)) > LINE_TOLERANCE && newObstacles.get(newObstacles.size()-1).getVertex(newObstacles.get(newObstacles.size()-1).size()) != obstacles.get(i).getVertex(u)) {
            if (distanceToLine(pa, obstacles.get(i).getVertex(j), obstacles.get(i).getVertex(u)) > LINE_TOLERANCE && lA != obstacles.get(i).getVertex(u)) {

              

              //newObstacles.get(i).addVertex(obstacles.get(i).getVertex(u).x, obstacles.get(i).getVertex(u).y);
              newObstacles.get(newObstacles.size()-1).addVertex(obstacles.get(i).getVertex(u).x, obstacles.get(i).getVertex(u).y);
              lA = obstacles.get(i).getVertex(u);
              a = j-1;
              //pa = obstacles.get(i).getVertex(a);
              //newObstacles.add(obstacles.get(i).getVertex(u));


            }


          }


        }


        if (newObstacles.get(newObstacles.size()-1).size() < 3) {
          newObstacles.remove(newObstacles.size()-1);
        }


      }



    }

    
    
	obstacles = newObstacles;
	reduced = true;
  }
  
  // Open up a dialog box and get the name of a file and then save the vector map to that file.
  public void save() {
    JFileChooser chooser = new JFileChooser(new File("."));
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Vector Map File", "vmp");
    chooser.setFileFilter(filter);
    int returnVal = chooser.showSaveDialog(null);
    if (returnVal == JFileChooser.APPROVE_OPTION) {
      String fileName = chooser.getSelectedFile().getName();
      if (!fileName.toUpperCase().endsWith(".VMP"))
        fileName += ".vmp";
      try {
        DataOutputStream theFile  = new DataOutputStream(new FileOutputStream(fileName));
        theFile.writeInt(width);
        theFile.writeInt(height);
        theFile.writeDouble(resolution);
        theFile.writeInt(obstacles.size());
        for (Obstacle ob:obstacles) {
          theFile.writeInt(ob.getVertices().size());
          for (Point p: ob.getVertices()) {
            theFile.writeInt(p.x);
            theFile.writeInt(p.y);
          }
        }
        theFile.close();
      }
      catch(IOException e) {
        JOptionPane.showMessageDialog(null, "Error saving vector map file", "Error", JOptionPane.ERROR_MESSAGE);
      }
    }
  }
  
  // Open up a dialog box and get the name of a file and then load up the vector map from that file.
  public void load() {
    JFileChooser chooser = new JFileChooser(new File("."));
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Vector Map File", "vmp");
    chooser.setFileFilter(filter);
    int returnVal = chooser.showOpenDialog(null);
    if (returnVal == JFileChooser.APPROVE_OPTION) {
      String fileName = chooser.getSelectedFile().getName();
      if (new File(fileName).exists()) {
        try {
          DataInputStream theFile  = new DataInputStream(new FileInputStream(fileName));
          int newWidth = theFile.readInt();
          int newHeight = theFile.readInt();
          double newRes = theFile.readDouble();
          if ((width != newWidth) || (height != newHeight) || (resolution != newRes)) {
            JOptionPane.showMessageDialog(null, "Error: Vector Map has different dimensions than current world","Error", JOptionPane.ERROR_MESSAGE);
            theFile.close();
            return;
          }
          obstacles = new ArrayList<Obstacle>();
          int count = theFile.readInt();
          for (int i=0; i<count; i++) {
            Obstacle ob = new Obstacle();
            int points = theFile.readInt();
            for (int j=0; j<points; j++) { 
              Point p = new Point(theFile.readInt(),theFile.readInt());
              ob.getVertices().add(p);
            }
            obstacles.add(ob);
          }
          theFile.close();
        }
        catch(IOException e) {
          JOptionPane.showMessageDialog(null, "Error loading vector map file", "Error", JOptionPane.ERROR_MESSAGE);
        }
      }
      else
        JOptionPane.showMessageDialog(null, "Error: vector map file does not exist: " + fileName, "Error", JOptionPane.ERROR_MESSAGE);
    }
  }
  
  
  // Draw the vector map
  public void draw(Graphics aPen, int magnification) {
    try {
      // Display the obstacles
      //if (!reduced) return;  /// Do not display the map if not yet reduced
      for (Obstacle ob: obstacles) {
        ArrayList<Point>  vertices = ob.getVertices();
        
        // Draw the edges first
        for (int i=0; i<vertices.size()-1; i++) {
            aPen.setColor(EdgeColor);
            aPen.drawLine(MapperApp.MARGIN_X/2 + vertices.get(i).x, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y, 
                          MapperApp.MARGIN_X/2 + vertices.get(i+1).x, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i+1).y);
        }
        aPen.drawLine(MapperApp.MARGIN_X/2 + vertices.get(0).x, 
                      MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(0).y, 
                      MapperApp.MARGIN_X/2 + vertices.get(vertices.size()-1).x, 
                      MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(vertices.size()-1).y);
        
        // Now Draw the vertices
        for (int i=0; i<vertices.size(); i++) {
            aPen.setColor(VertexColor);
            aPen.fillOval(MapperApp.MARGIN_X/2 + vertices.get(i).x-VERTEX_RADIUS, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y-VERTEX_RADIUS, 
                          VERTEX_DIAMETER, VERTEX_DIAMETER);
            aPen.setColor(Color.black);
            aPen.drawOval(MapperApp.MARGIN_X/2 + vertices.get(i).x-VERTEX_RADIUS, 
                          MapperApp.MARGIN_Y/2 + height*magnification - vertices.get(i).y-VERTEX_RADIUS, 
                          VERTEX_DIAMETER, VERTEX_DIAMETER);
        }
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }

}