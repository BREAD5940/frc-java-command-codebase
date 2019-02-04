package frc.robot.lib;

import java.util.ArrayList;

/**
 * A class for filtering the contours of vision targets posted to network tables
 * by a grip pipeline running on the limelight. For now only supports
 */
public class ContourFiltering {

  NetworkTableInterface table;
  
  ArrayList<String> keys = new ArrayList<String>() {
    { 
      add("centerX"); 
      add("centerY"); 
      add("area"); 
      add("area"); 
      add("area"); 
    } 
  };




  public ContourFiltering() {
    table = new NetworkTableInterface("GRIP/contourReport");
  }

  public ArrayList<Double> getData() {
    return updateTableEntries();
  }

  private ArrayList<Double> updateTableEntries() {
    ArrayList<Double> entries;
    return null;
  }

}