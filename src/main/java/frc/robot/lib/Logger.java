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

  public static void groupStart(String name) {
    Logger.groupNames.add(name);
    Logger.i++;
    Logger.out(Colors.orange + "<" + Colors.red + name + Colors.orange + ">");
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

  public static void group(String name, Runnable fn) {
    Logger.groupStart(name);
    fn.run();
    Logger.groupEnd();
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
    System.out.print(Colors.reset);
  }

  public static void log(String s) { Logger.out(s); }
  
  public static void log(Object o) { Logger.warn("Logger.log does not support logging Objects yet"); }
  // TODO actually make this work. Maybe toPairs.java?
  public static void warn(String s) { Logger.out(Colors.yellow + s); }
  public static void error(String s) { Logger.out(Colors.red + s); }
  /** Will interpolate using BashInterpolation.java and will then log */
  public static void interlog(String s, Object o) { Logger.out($.i(s, o)); }
  public static void interlog(String s, String[] ary) { Logger.out($.i(s, ary)); }
  public static void interlog(String s, ArrayList<String> ary) { Logger.out($.i(s, ary)); }

  public static void colorlog(String s) { Logger.log($.i(s, Colors.class)); }
  /**
   * Run a method that returns void and takes 0 arguments in a try-catch block.
   * <pre>Logger.runSafe(MyClass::MyMethod);</pre>
   * @param fn An Object that extends Runnable
   */
  public static boolean runSafe(Runnable fn) {
    try {
      fn.run();
    } catch (Exception e) {
      Logger.error("robot.lib.Logger.runSafe: Something bad happened");
      return false;
    }
    return true;
  }
}
