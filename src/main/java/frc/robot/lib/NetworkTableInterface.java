//package frc.robot.lib;
//
//import java.util.ArrayList;
//
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//
//public class NetworkTableInterface {
//	NetworkTable mTable;
//	ArrayList<String> keys;
//	// private ArrayList<NetworkTableEntry> networkTableEntries;
//
//	public NetworkTableInterface(String database, ArrayList<String> keys) {
//		NetworkTableInstance nt = NetworkTableInstance.getDefault();
//		mTable = nt.getTable(database);
//		this.keys = keys;
//	}
//
//	public NetworkTableInterface(String database) {
//		NetworkTableInstance nt = NetworkTableInstance.getDefault();
//		mTable = nt.getTable(database);
//	}
//
//	public NetworkTable getTable() {
//		return mTable;
//	}
//
//	public NetworkTableEntry getEntry(String key) {
//		return mTable.getEntry(key);
//	}
//
//	public void setEntry(String key, double value) {
//		mTable.getEntry(key).setDouble(value);
//	}
//
//	public void setEntry(String key, boolean value) {
//		mTable.getEntry(key).setBoolean(value);
//	}
//
//	public void setEntry(String key, double[] value) {
//		mTable.getEntry(key).setDoubleArray(value);
//	}
//
//	public void setEntry(String key, String value) {
//		mTable.getEntry(key).setString(value);
//	}
//
//}
