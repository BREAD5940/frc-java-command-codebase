package frc.robot.lib;

import java.util.ArrayList;
import java.lang.reflect.Field;
import java.util.regex.Pattern;
import java.util.function.Function;
import java.lang.RuntimeException;

/**
 * Java interpolation doesn't HAVE to suck
 * It's so you can do stuff like this:
 * "hello${foobar} world"
 * @author Cole Gannon
 */
public class BashInterpolation {
  private static Pattern $pattern = Pattern.compile("\\${?(\\w+)}?");

  /**
   * @param s Input string to search for matches
   * @param fn It's a thing
   */
  // might be able to use this elsewhere since it's general but who cares really
  private static String replaceAll(String s, Function<String, String> fn) {
    var matcher = BashInterpolation.$pattern.matcher(s);
    var foundAtLeastOne = false;
    var res = "";
    var lastEnd = 0;
    while (matcher.find()) {
      foundAtLeastOne = true;
      var start = matcher.start();
      res += s.substring(lastEnd, start);
      lastEnd = matcher.end();
      res += fn.apply(matcher.group(1));
    }
    if (!foundAtLeastOne) {
      return s;
    }
    return res;
  }
  private String result;
  /** Get the result */
  public String result() {
    return this.result;
  }
  /** Gets an environment variable. Probably not what you want to do */
  public BashInterpolation(String s) {
    this.result = BashInterpolation.replaceAll(s, System::getenv);
  }
  public BashInterpolation(String s, String[] ary) {
    this.result = BashInterpolation.replaceAll(s, (String match) -> {
      int i;
      try {
        i = Integer.parseInt(match);
      } catch (Exception e) {
        throw new RuntimeException("BashInterpolation: Cannot interpolate " + match + " from a Array<String>!");
      }
      return ary[i];
    });
  }
  public BashInterpolation(String s, ArrayList<String> ary) {
    this.result = BashInterpolation.replaceAll(s, (String match) -> {
      int i;
      try {
        i = Integer.parseInt(match);
      } catch (Exception e) {
        throw new RuntimeException("BashInterpolation: Cannot interpolate " + match + " from a ArrayList<String>!");
      }
      return ary.get(i);
    });
  }
  /** Given a String and an input Object */
  public BashInterpolation(String s, Object o) {
    Class<?> cls = o.getClass();
    this.result = BashInterpolation.replaceAll(s, (String match) -> {
      try {
        Field f = cls.getField(match);
        if (f.getType().equals(String.class)) {
          return (String) f.get(o);
        }
      } catch (Exception e) {
        throw new RuntimeException("Couldn't access " + match + " on the provided Object");
      }
      throw new RuntimeException("Found " + match + " but it was not a String!");
    });
  }
  /** Interpolate using static fields from a Class */
  public BashInterpolation(String s, Class<?> c) {
    this.result = BashInterpolation.replaceAll(s, (String match) -> {
      try {
        Field f = c.getField(match);
        if (f.getType().equals(String.class)) {
          return (String) f.get(null);
        }
      } catch (Exception e) {
        throw new RuntimeException("Couldn't access " + match + " on the provided Object");
      }
      throw new RuntimeException("Found " + match + " but it was not a String!");
    });
  }
}
