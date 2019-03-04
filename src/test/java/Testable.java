
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Testable {

	public void writeToCSV(String file, double[][] path) {

		try {
			FileWriter fw = new FileWriter(file);
			PrintWriter pw = new PrintWriter(fw, true);

			pw.println("time,elevatorHeight,drivetrainSpeed");
			for (double[] t : path) {
				pw.println(t[0] + "," + t[1] + "," + t[2]);
			}

			// pw.print("adsffdsaadsfdsfaadsffads1");

			pw.close();
		} catch (IOException ioe) {
			System.out.println(ioe);
		}

	}

}
