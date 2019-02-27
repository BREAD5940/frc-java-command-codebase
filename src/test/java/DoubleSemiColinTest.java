import java.util.function.Consumer;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.junit.jupiter.api.Test;

public class DoubleSemiColinTest {

  public double meme;

  public void setMeme(double new_) {
    this.meme = new_;
  }

  public class TestMeme {
    Consumer<Double> testC;
    public TestMeme(Consumer<Double> test) {
      this.testC = test;
    }
    public void run() {
      testC.accept(Math.random() * 55);
    }
  }

  @Test
  public void testDoubleSemiColin() {
    System.out.println("current val: " + meme);
    TestMeme memeTest = new TestMeme(this::setMeme);
    System.out.println("current val: " + meme);
    memeTest.run();
    System.out.println("current val: " + meme);
    
  }

}