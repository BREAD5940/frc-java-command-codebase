

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.math.coordinateSystems;

import org.junit.jupiter.api.Test;


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
        double[] calcQ1p = coordinateSystems.polarToCartesian(polarQuadrent1positiveMagnitude);
        double[] calcQ1n = coordinateSystems.polarToCartesian(polarQuadrent1negativeMagnitude);
        double[] calcQ2p = coordinateSystems.polarToCartesian(polarQuadrent2positiveMagnitude);
        double[] calcQ2n = coordinateSystems.polarToCartesian(polarQuadrent2negativeMagnitude);
        double[] calcQ3p = coordinateSystems.polarToCartesian(polarQuadrent3positiveMagnitude);
        double[] calcQ3n = coordinateSystems.polarToCartesian(polarQuadrent3negativeMagnitude);
        double[] calcQ4p = coordinateSystems.polarToCartesian(polarQuadrent4positiveMagnitude);
        double[] calcQ4n = coordinateSystems.polarToCartesian(polarQuadrent4negativeMagnitude);

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

        double actual1 = coordinateSystems.calculateChordLen(input1[0], input1[1]);
        double actual2 = coordinateSystems.calculateChordLen(input2[0], input2[1]);
        double actual3 = coordinateSystems.calculateChordLen(input3[0], input3[1]);

        assertEquals(expected1, actual1, 0.01);
        assertEquals(expected2, actual2, 0.01);
        assertEquals(expected3, actual3, 0.01);

    }

    @Test
    public void testDisplacement() {
        // calculaeDisplacement(double deltaLeft, double deltaRight, double oldAngle, double currentAngle) 

        // This test case will check what happens if the radii are infinity
        double[] input1 = {1, 1, 0, 0};
        double[] input2 = {1, 2, 0, -28};
        double[] input3 = {1.5, 2.5, 5, 28};

        double[] expected1 =  {0, 1};
        double[] expected2 =  {-0.357, 1.441};
        double[] expected3 =  {0.56, 1.905};


        double[] actual1 = coordinateSystems.calculaeDisplacement(input1[0], input1[1], input1[2], input1[3]);
        double[] actual2 = coordinateSystems.calculaeDisplacement(input2[0], input2[1], input2[2], input2[3]);
        double[] actual3 = coordinateSystems.calculaeDisplacement(input3[0], input3[1], input3[2], input3[3]);

        assertArrayEquals(expected1, actual1, 0.02);
        assertArrayEquals(expected2, actual2, 0.02);
        assertArrayEquals(expected3, actual3, 0.02);


    }


}