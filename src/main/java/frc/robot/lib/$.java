package frc.robot.lib;

import java.util.ArrayList;
import java.lang.reflect.Field;
import java.util.regex.Pattern;
import java.util.function.Function;
import java.lang.RuntimeException;

/**
 * Java interpolation doesn't <i>have</i> to suck.
 * This class is a loose implementation of bash's $ variable syntax.
 * @author Cole Gannon
 */
public class $ {
  // go ahead and complain that "$" for the name is too terse but it'll save you a lot of typing
  // also, if you're gonna read this file it'd be worth reading up on lambdas
  /** A Regex designed to get variable names from bash <code>$</code> expression compiled to a Pattern */
  private static Pattern $pattern = Pattern.compile("\\${?(\\w+)}?");
  /**
   * Replace every $<variable> with the output of applying <variable> to fn
   * @param s Input string to search for matches
   * @param fn A function that bash variables will be applied to.
   * It returns a String which the bash <code>$</code> expression will be replaced with
   */
  // might be able to use this elsewhere since it's general but who cares really
  private static String replaceAll(String s, Function<String, String> fn) {
    var matcher = $.$pattern.matcher(s);
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
  /** Gets environment variables as specified by <code>$</code>. */
  public $(String s) {
    this.result = $.replaceAll(s, System::getenv);
  }
  /**
   * Interpolate given a String and an Array of Strings.
   * @param s The input String
   * @param ary The Array of variables to access
   */
  public $(String s, String[] ary) {
    this.result = $.replaceAll(s, (String match) -> {
      int i;
      try {
        i = Integer.parseInt(match);
      } catch (Exception e) {
        throw new RuntimeException("BashInterpolation: Cannot interpolate " + match + " from a Array<String>!");
      }
      return ary[i];
    });
  }
  /**
   * Interpolate given a String and an ArrayList&lt;String&gt;
   * @param s The input String
   * @param ary The ArrayList of Strings to access
   */
  public $(String s, ArrayList<String> ary) {
    this.result = $.replaceAll(s, (String match) -> {
      int i;
      try {
        i = Integer.parseInt(match);
      } catch (Exception e) {
        throw new RuntimeException("BashInterpolation: Cannot interpolate " + match + " from a ArrayList<String>!");
      }
      return ary.get(i);
    });
  }
  /**
   * Interpolate given a String and an input Object
   * @param s The input String
   * @param o Literally any Java Object
   */
  public $(String s, Object o) {
    Class<?> cls = o.getClass();
    this.result = $.replaceAll(s, (String match) -> {
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
  public $(String s, Class<?> c) {
    this.result = $.replaceAll(s, (String match) -> {
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
  /**
   * Faster way to interpolate <code>$</code> environment variables
   * @param s Input
   */
  public static String i(String s) { return new $(s).result(); }
  /**
   * Given a String and any Object. Example:
   * <pre>
   *String foo = $.i(
   *  "Java is as fast as a $noun",
   *  new Object() {
   *    static String animal = "sloth";
   *  }
   *);
   * </pre>
   * @param s The input String
   * @param o Literally any Object
   */
  public static String i(String s, Object o) { return new $(s, o).result(); }
  /**
   * Given a String and an Array of Strings. Example:
   * <pre>
   *String bar = $.i(
   *  "It ain't much but it's $0 $1",
   *  {"honest", "work"}
   *);
   * // yes, it's zero indexed
   * </pre>
   * @param s The input String
   * @param ary The Array of variables to access
   */
  public static String i(String s, String[] ary) { return new $(s, ary).result(); }
  /**
   * Given a String and an ArrayList of Strings. Example:
   * <pre>
   *var nouns = new ArrayList&lt;String&gt;();
   *nouns.add("bacon");
   *nouns.add("calorie counts");
   *String baz = $.i("With great $0 comes great $1", nouns);
   * </pre>
   * @param s The input String
   * @param ary The ArrayList of Strings to access
   */
  public static String i(String s, ArrayList<String> ary) { return new $(s, ary).result(); }
}
