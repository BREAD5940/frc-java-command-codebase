package frc.robot.lib;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableInterface {
  NetworkTable mTable;
  private int databaseLen;
  // private ArrayList<NetworkTableEntry> networkTableEntries;

  public NetworkTableInterface(String database, String[] keys) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    mTable = nt.getTable(database);
    databaseLen = keys.length;
    // for(int i=0; i<databaseLen; i++) {
    //   networkTableEntries.add( mTable.getEntry(keys[i]) );
    // }
  }

  public NetworkTableEntry getEntry(String key) {
    return mTable.getEntry(key);
  }

}