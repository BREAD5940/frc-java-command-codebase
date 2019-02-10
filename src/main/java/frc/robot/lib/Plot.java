package frc.robot.lib;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

import javax.swing.*;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;

// import processing.core.*;

public class Plot extends JFrame {
	JFrame frame = new JFrame();
	JPanel panelBgImg;

	ArrayList<Integer> topLeft = new ArrayList<Integer>(Arrays.asList(217, 40));
	ArrayList<Integer> bottomRight = new ArrayList<Integer>(Arrays.asList(1372, 615));
	ArrayList<Integer> fieldSize = new ArrayList<Integer>(Arrays.asList(54, 27));
	ArrayList<Integer> fieldSizeInPixels = new ArrayList<Integer>(Arrays.asList((bottomRight.get(0) - topLeft.get(0)), (bottomRight.get(1) - topLeft.get(1))));

	/**
	* Convert a x,y double in field coordinates into pixels on the field image
	* @param 
	*/
	public Translation2d convertToPixels(Translation2d feet) {
		return new Translation2d(Math.round(feet.getX().getFeet() / fieldSize.get(0) * fieldSizeInPixels.get(0) + topLeft.get(0)),
				Math.round(feet.getY().getFeet() / fieldSize.get(1) * fieldSizeInPixels.get(1) + topLeft.get(1)));

	}

	public double ctp(double feet, boolean isY) {
		if (isY) {
			return feet / fieldSize.get(1) * fieldSizeInPixels.get(1) + topLeft.get(1);
		} else {
			return feet / fieldSize.get(0) * fieldSizeInPixels.get(0) + topLeft.get(0);
		}
	}

	public Plot(double[][] whee, double[][] unWhee) {
		setTitle("Path Visualizer");
		for (int i = 0; i < whee.length; i++) {
			JButton button = new JButton();

			button.setBounds((int) Math.round(ctp(whee[i][0], false)), (int) Math.round(ctp(whee[i][1], true)), 10, 10);
			frame.add(button);
		}

		for (int i = 0; i < unWhee.length; i++) {
			JButton button = new JButton();

			button.setBounds((int) Math.round(ctp(unWhee[i][0], false)), (int) Math.round(ctp(unWhee[i][1], true)), 5, 5);
			frame.add(button);
		}
		//   for(int i=0; i<roughPts.size(); i++){
		//     JButton button  = new JButton();
		//     // System.out.print(i);
		//     System.out.print(",");
		//     Translation2d toPrint = convertToPixels(roughPts.get(i));
		//     button.setBounds((int)Math.round(toPrint.x()), (int)Math.round(toPrint.y()), 5,5);
		//     frame.add(button); // adds button in JFrame 
		//   }
		//   System.out.println();
		//   ArrayList<Translation2d> smoothPts = Visualizer.smoother(roughPts, a, b, c);
		//   for(int i=0; i<smoothPts.size(); i++){
		//     JButton button  = new JButton();
		//     // System.out.print(i);
		//     System.out.print(",");
		//     Translation2d toPrint = convertToPixels(smoothPts.get(i));
		//     button.setBounds((int)Math.round(toPrint.x()), (int)Math.round(toPrint.y()), 10,10);
		//     frame.add(button); // adds button in JFrame 
		//   }

		// JLabel label = new JLabel("UserName: ", JLabel.LEFT);

		ImageIcon imh = new ImageIcon("src\\main\\java\\pantryvisualizer\\2019-field.jpg");
		setSize(imh.getIconWidth(), imh.getIconHeight());

		panelBgImg = new JPanel() {
			public void paintComponent(Graphics g) {
				Image img = new ImageIcon("src\\main\\java\\pantryvisualizer\\2019-field.jpg").getImage();
				Dimension size = new Dimension(img.getWidth(null), img.getHeight(null));
				setPreferredSize(size);
				setMinimumSize(size);
				setMaximumSize(size);
				setSize(size);
				setLayout(null);
				g.drawImage(img, 0, 0, null);
			}
		};

		frame.add(panelBgImg);
		panelBgImg.setBounds(0, 0, imh.getIconWidth(), imh.getIconHeight());

		// GridBagLayout layout = new GridBagLayout();

		// JPanel panelContent = new JPanel(layout);
		// GridBagConstraints gc = new GridBagConstraints();

		// gc.insets = new Insets(3, 3, 3, 3);
		// gc.gridx = 1;
		// gc.gridy = 1;

		// JLabel label = new JLabel("UserName: ", JLabel.LEFT);                        
		// panelContent.add(label, gc);

		// gc.gridx = 2;
		// gc.gridy = 1;

		// JTextField txtName = new JTextField(10);
		// panelContent.add(txtName, gc);

		// gc.insets = new Insets(3, 3, 3, 3);
		// gc.gridx = 1;
		// gc.gridy = 2;
		// gc.gridwidth = 2;

		// panelBgImg.add(panelContent);

		// panelBgImg.setLayout(new FlowLayout(FlowLayout.CENTER, 150, 200));

		setResizable(false);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); // setting close operation 

		frame.setSize(imh.getIconWidth(), imh.getIconHeight()); // sets 500 width and 600 height 

		frame.setLayout(null); // uses no layout managers

		frame.setVisible(true); // makes the frame visible 
	}

	//   public Plot(ArrayList<Translation2d> points){ 
	//         setTitle("Path Visualizer");
	//         for (Translation2d i : points){
	//             JButton button  = new JButton();
	//             i = convertToPixels(i);
	//             System.out.print(i);
	//             System.out.print(",");
	//             button.setBounds((int)Math.round(i.x()), (int)Math.round(i.y()), 10,10);
	//             frame.add(button); // adds button in JFrame 
	//         }    

	// // JLabel label = new JLabel("UserName: ", JLabel.LEFT);

	// ImageIcon imh = new ImageIcon("src\\main\\java\\pantryvisualizer\\2019-field.jpg");
	// setSize(imh.getIconWidth(), imh.getIconHeight());

	// panelBgImg = new JPanel()
	// {
	//     public void paintComponent(Graphics g) 
	//     {
	//         Image img = new ImageIcon("src\\main\\java\\pantryvisualizer\\2019-field.jpg").getImage();
	//         Dimension size = new Dimension(img.getWidth(null), img.getHeight(null));
	//         setPreferredSize(size);
	//         setMinimumSize(size);
	//         setMaximumSize(size);
	//         setSize(size);
	//         setLayout(null);
	//         g.drawImage(img, 0, 0, null);
	//     } 
	// };

	// frame.add(panelBgImg);
	// panelBgImg.setBounds(0, 0, imh.getIconWidth(), imh.getIconHeight());

	// // GridBagLayout layout = new GridBagLayout();

	// // JPanel panelContent = new JPanel(layout);
	// // GridBagConstraints gc = new GridBagConstraints();

	// // gc.insets = new Insets(3, 3, 3, 3);
	// // gc.gridx = 1;
	// // gc.gridy = 1;

	// // JLabel label = new JLabel("UserName: ", JLabel.LEFT);                        
	// // panelContent.add(label, gc);

	// // gc.gridx = 2;
	// // gc.gridy = 1;

	// // JTextField txtName = new JTextField(10);
	// // panelContent.add(txtName, gc);

	// // gc.insets = new Insets(3, 3, 3, 3);
	// // gc.gridx = 1;
	// // gc.gridy = 2;
	// // gc.gridwidth = 2;

	// // panelBgImg.add(panelContent);

	// // panelBgImg.setLayout(new FlowLayout(FlowLayout.CENTER, 150, 200));

	// setResizable(false);
	// setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

	// frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); // setting close operation 

	// frame.setSize(imh.getIconWidth(), imh.getIconHeight()); // sets 500 width and 600 height 

	// frame.setLayout(null); // uses no layout managers

	// frame.setVisible(true); // makes the frame visible 
	// } 
}
