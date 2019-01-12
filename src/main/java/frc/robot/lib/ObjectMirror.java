package frc.robot.lib;

import java.lang.reflect.Field;

/**
 * Hah, get it since it uses reflection?
 * @author Cole Gannon
 */
public class ObjectMirror {
  class KeyValue<K, V> {
    K key;
    V value;
  }
  
  public static KeyValue[] fieldsToPairs(Object o) {
    Class<?> cls = o.getClass();
  }
}
