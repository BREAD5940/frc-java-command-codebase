package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableInterface {
  NetworkTable mTable;
  // private ArrayList<NetworkTableEntry> networkTableEntries;

  public NetworkTableInterface(String database, String[] keys) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    mTable = nt.getTable(database);
    // for(int i=0; i<databaseLen; i++) {
    //   networkTableEntries.add( mTable.getEntry(keys[i]) );
    // }
  }

  public NetworkTableEntry getEntry(String key) {
    return mTable.getEntry(key);
  }

  public void setEntry(String key, double value) {
    mTable.getEntry(key).setDouble(value);
  }

  public void setEntry(String key, boolean value) {
    mTable.getEntry(key).setBoolean(value);
  }

  public void setEntry(String key, double[] value) {
    mTable.getEntry(key).setDoubleArray(value);
  }

  public void setEntry(String key, String value) {
    mTable.getEntry(key).setString(value);
  }

}