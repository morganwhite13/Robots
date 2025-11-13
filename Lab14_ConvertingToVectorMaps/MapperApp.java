//Author Morgan White SN: 101184799 & Coauthor: Jacob Harper SN: 101111173

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import java.io.*;
import java.awt.Dimension;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

public class MapperApp extends JFrame {
  private static final long serialVersionUID = 99999999;
  
  static final float Resolution = 0.5f; // 0.5 cm per grid cell
  static final int Magnification = 2;  // 2x magnification
  static int WindowWidth;
  static int WindowHeight;
  static int MARGIN_X = 10;
  static int MARGIN_Y = 10;
  
  private MapperPanel  mapPanel;

  public MapperApp(int widthInCentimeters, int heighInCentimeters) {
    super("Mapper App");

    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification)+ MARGIN_X*2;
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification) + MARGIN_Y*2;
    
    add(mapPanel = new MapperPanel((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution, Magnification));


    // Create the menu bar and menus
    JMenuBar menuBar = new JMenuBar();
    setJMenuBar(menuBar);
    
    // Set up the Grid Map menu
    JMenu 	  mapMenu = new JMenu("GridMap"); menuBar.add(mapMenu);
    JMenuItem  loadGridMapButton = new JMenuItem("Load Grid Map");
    mapMenu.add(loadGridMapButton);
    loadGridMapButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map m = mapPanel.mapFromFile();
        WindowWidth = (int)(m.getWidth()/Resolution)+ MARGIN_X*2;
        WindowHeight = (int)(m.getHeight()/Resolution) + MARGIN_Y*2;
        setSize(WindowWidth+4, WindowHeight+52);
        recompute();
        mapPanel.update();
      }});

    // Set up the Binary Threshold menu
    JMenu	  thresholdMenu = new JMenu("BinaryThreshold"); menuBar.add(thresholdMenu);
    JCheckBoxMenuItem showBinaryButton = new JCheckBoxMenuItem("Show Binary Map");  thresholdMenu.add(showBinaryButton);
    SpinnerModel model = new SpinnerNumberModel(0, 0, 255, 1);
    JSpinner  binarySpinner = new JSpinner(model);  thresholdMenu.add(binarySpinner);
    JCheckBoxMenuItem showBordersButton = new JCheckBoxMenuItem("Show Borders");  thresholdMenu.add(showBordersButton);
    showBinaryButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowBinary = showBinaryButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    showBordersButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowBorders = showBordersButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    binarySpinner.addChangeListener(new ChangeListener() {
      public void stateChanged(ChangeEvent e) {
        Map.BINARY_THRESHOLD = (int)(binarySpinner.getValue());
        recompute();
        mapPanel.update();
      }});
      
    // Set up the Vector Map menu
    /*JMenu 	    vectorMapMenu = new JMenu("VectorMap"); menuBar.add(vectorMapMenu);
    JMenuItem  saveVecMapButton = new JMenuItem("Save Vector Map");
    JMenuItem  loadVecMapButton = new JMenuItem("Load Vector Map");
    vectorMapMenu.add(saveVecMapButton); vectorMapMenu.add(loadVecMapButton);
    saveVecMapButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        mapPanel.getMap().getVectorMap().save();
      }});
    loadVecMapButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        mapPanel.getMap().getVectorMap().load();
        mapPanel.update();
      }});*/

    // Set up the Line Tolerance Menu
    JMenu	  toleranceMenu = new JMenu("LineTolerance"); menuBar.add(toleranceMenu);
    JCheckBoxMenuItem hideGrayButton = new JCheckBoxMenuItem("Hide Grayscale Map");  toleranceMenu.add(hideGrayButton);
    JCheckBoxMenuItem showVectorButton = new JCheckBoxMenuItem("Show Vector Map");  toleranceMenu.add(showVectorButton);
    SpinnerModel model2 = new SpinnerNumberModel(0, 0, 50, 1);
    JSpinner  toleranceSpinner = new JSpinner(model2);  toleranceMenu.add(toleranceSpinner);
    hideGrayButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        Map.ShowGrayscale = !hideGrayButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    showVectorButton.addActionListener(new ActionListener() {
      public void actionPerformed(ActionEvent e) {
        VectorMap.ShowVector = showVectorButton.isSelected();
        recompute();
        mapPanel.update();
      }});
    toleranceSpinner.addChangeListener(new ChangeListener() {
      public void stateChanged(ChangeEvent e) {
        VectorMap.LINE_TOLERANCE = (int)(toleranceSpinner.getValue());
        recompute();
        mapPanel.update();
      }});

    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setSize(WindowWidth+4, WindowHeight+52);
    setVisible(true);
  }
  
  // Apply a sensor model reading to the map
  void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance, double beamWidthInDegrees, double distanceErrorAsPercent) { 
    mapPanel.getMap().applySensorModelReading(sensorX, 
                                              sensorY,
                                              sensorAngle,
                                              distance,
                                              beamWidthInDegrees,
                                              distanceErrorAsPercent);
    recompute();
    mapPanel.update();
  }
  
  // Recompute the binary grid and vector map if needed
  void recompute() { 
    try {
      if (Map.ShowBorders || Map.ShowBinary || VectorMap.ShowVector) 
        mapPanel.getMap().computeBinaryGrid();
      if (Map.ShowBorders || VectorMap.ShowVector) 
        mapPanel.getMap().computeBorders();
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
  
  // Update the grid by applying the temp grid changes to the occupancy grid
  void updateGrid(double x, double y) { 
    mapPanel.getMap().updateGrid(x, y);
    recompute();
    mapPanel.update();
  }
  
  public static void main(String[] args) {
    MapperApp  mapper = new MapperApp(200, 150);
  }
}
