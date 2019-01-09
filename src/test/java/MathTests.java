import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import java.util.ArrayList;

import frc.math.CoordinateSystems;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.DriveTrain;

public class MathTests {

    @Test
    /** Is this overkill? I think not */
    public void testPolarToCartesian() {
        // Odometry coordinateSystems = new Odometry();
        double[] polarQuadrent1positiveMagnitude = {45, 2};
        double[] polarQuadrent1negativeMagnitude = {45, -2};
        double[] polarQuadrent2positiveMagnitude = {135, 2};
        double[] polarQuadrent2negativeMagnitude = {135, -2};
        double[] polarQuadrent3positiveMagnitude = {200, 2};
        double[] polarQuadrent3negativeMagnitude = {200, -2};
        double[] polarQuadrent4positiveMagnitude = {320, 2};
        double[] polarQuadrent4negativeMagnitude = {320, -3};
        double[] expectedQ1p = {1.414, 1.414};
        double[] expectedQ1n = {-1.414, -1.414}; 
        double[] expectedQ2p = {-1.414,1.414};
        double[] expectedQ2n = {1.414, -1.414};
        double[] expectedQ3p = {-1.879, -0.684};
        double[] expectedQ3n = {1.879, 0.684};
        double[] expectedQ4p = {1.532, -1.286};
        double[] expectedQ4n = {-2.298, 1.928};
        double[] calcQ1p = CoordinateSystems.polarToCartesian(polarQuadrent1positiveMagnitude);
        double[] calcQ1n = CoordinateSystems.polarToCartesian(polarQuadrent1negativeMagnitude);
        double[] calcQ2p = CoordinateSystems.polarToCartesian(polarQuadrent2positiveMagnitude);
        double[] calcQ2n = CoordinateSystems.polarToCartesian(polarQuadrent2negativeMagnitude);
        double[] calcQ3p = CoordinateSystems.polarToCartesian(polarQuadrent3positiveMagnitude);
        double[] calcQ3n = CoordinateSystems.polarToCartesian(polarQuadrent3negativeMagnitude);
        double[] calcQ4p = CoordinateSystems.polarToCartesian(polarQuadrent4positiveMagnitude);
        double[] calcQ4n = CoordinateSystems.polarToCartesian(polarQuadrent4negativeMagnitude);

        // Check all 16 indexes of the expected vs actual cartesial coordinates
        assertEquals( expectedQ1p[0], calcQ1p[0], 0.01 );
        assertEquals( expectedQ1p[1], calcQ1p[1], 0.01 );
        assertEquals( expectedQ1n[0], calcQ1n[0], 0.01 );
        assertEquals( expectedQ1n[1], calcQ1n[1], 0.01 );

        assertEquals( expectedQ2p[0], calcQ2p[0], 0.01 );
        assertEquals( expectedQ2p[1], calcQ2p[1], 0.01 );
        assertEquals( expectedQ2n[0], calcQ2n[0], 0.01 );
        assertEquals( expectedQ2n[1], calcQ2n[1], 0.01 );
        
        assertEquals( expectedQ3p[0], calcQ3p[0], 0.01 );
        assertEquals( expectedQ3p[1], calcQ3p[1], 0.01 );
        assertEquals( expectedQ3n[0], calcQ3n[0], 0.01 );
        assertEquals( expectedQ3n[1], calcQ3n[1], 0.01 );

        assertEquals( expectedQ4p[0], calcQ4p[0], 0.01 );
        assertEquals( expectedQ4p[1], calcQ4p[1], 0.01 );
        assertEquals( expectedQ4n[0], calcQ4n[0], 0.01 );
        assertEquals( expectedQ4n[1], calcQ4n[1], 0.01 );
        
    }

    @Test
    public void testChordLength() {
        double[] input1 = {2, 30};
        double[] input2 = {5, 162};
        double[] input3 = {3, 612};

        double expected1 =  1.035;
        double expected2 = 9.877;
        double expected3 = 4.854;

        double actual1 = CoordinateSystems.calculateChordLen(input1[0], input1[1]);
        double actual2 = CoordinateSystems.calculateChordLen(input2[0], input2[1]);
        double actual3 = CoordinateSystems.calculateChordLen(input3[0], input3[1]);

        assertEquals(expected1, actual1, 0.01);
        assertEquals(expected2, actual2, 0.01);
        assertEquals(expected3, actual3, 0.01);

    }

    @Test
    public void testDisplacement() {
        // calculaeDisplacement(double deltaLeft, double deltaRight, double oldAngle, double currentAngle) 

        int tests = 6;

        double[][] inputs = new double[tests][3];
        double[][] expecteds = new double[tests][3];
        double[][] calculateds = new double[tests][3];
        
        inputs[0] = new double[]{1, 1, 0, 0};
        inputs[1] = new double[]{1, 2, 0, -28};
        inputs[2] = new double[]{1.5, 2.5, 5, 28};
        inputs[3] = new double[]{2.5, 1.5, 5, 28};
        inputs[4] = new double[]{3, 4, 37, 10};
        inputs[5] = new double[]{3, 5, 34, -15};

        expecteds[0] = new double[]{0, 1};
        expecteds[1] = new double[]{-0.357, 1.441};
        expecteds[2] = new double[]{0.56, 1.905};
        expecteds[3] = new double[]{0.56, 1.905};
        expecteds[4] = new double[]{1.383, 3.18};
        expecteds[5] = new double[]{0.64, 3.828};

        for ( int i=0; i < tests; i++) {
            calculateds[i] = CoordinateSystems.calculaeDisplacement(inputs[i][0], inputs[i][1], inputs[i][2], inputs[i][3]);
            System.out.println("Calcualteds: " + calculateds[i][0] + ", " + calculateds[i][1]);
            System.out.println("Expecteds: " + expecteds[i][0] + ", " + expecteds[i][1]);

            assertArrayEquals(expecteds[i], calculateds[i], 0.02);
        }

    }


}
