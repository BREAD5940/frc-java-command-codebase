package frc.robot.lib;

import java.lang.reflect.*;
import java.lang.Runnable;
import java.util.ArrayList;

/**
 * A Logger class. This one's not about deforestation though.
 * @author Cole Gannon
 */
public class Logger {
  /** Last index of groupNames */
  private static int i = -1;
  private static ArrayList<String> groupNames = new ArrayList<String>();
  private static String indent = Colors.red + "| " + Colors.reset;

  public static void groupStart(String s) {
    Logger.groupNames.add(s);
    Logger.i++;
    Logger.out("<" + s + ">");
  }

  public static void groupStart() {
    Logger.groupStart("Group " + (i + 2));
  }

  public static void groupEnd() {
    if (Logger.i < 0) {
      // Logger.warn("");
    } else {
      var removedGroup = Logger.groupNames.get(Logger.i);
      Logger.out("</" + removedGroup + ">");
      Logger.groupNames.remove(Logger.i);
      Logger.i--;
    }
  }

  private static String computeIndentation() {
    String s = "";
    var i = Logger.i + 1;
    while (i-- > 0) { // read as "while i goes to zero"
      s += Logger.indent;
    }
    // also, Java should really have a (String or Character).repeat method
    return s;
  }

  private static void out(String s) {
    var ary = s.split("\\n\\r?"); // mfw there aren't any regex literals
    var indent = Logger.computeIndentation();
    for (var line : ary) {
      System.out.println(indent + line);
    }
  }

  public static void log(String s) { Logger.log(s); }
  
  public static void log(Object o) { }
  // TODO actually make this work. Maybe toPairs.java?
  
  /** Will interpolate using BashInterpolation.java and will then log */
  public static void interlog(String s, Object o) { Logger.log($.i(s, o)); }
  public static void interlog(String s, String[] ary) { Logger.log($.i(s, ary)); }
  public static void interlog(String s, ArrayList<String> ary) { Logger.log($.i(s, ary)); }
  /**
   * Run a method in a try-catch block. Logger.runSafe(MyClass:MyMethod);
   */
  public static void runSafe(Runnable fn) {
    try {
      fn.run();
    } catch (Exception e) {

    }
  }
}
