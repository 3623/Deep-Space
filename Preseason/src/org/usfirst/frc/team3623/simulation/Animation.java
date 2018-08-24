package org.usfirst.frc.team3623.simulation;

import javax.imageio.ImageIO;
import javax.swing.*;

import org.usfirst.frc.team3623.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3623.robot.util.CartesianCoordinate;
import org.usfirst.frc.team3623.robot.util.Tuple;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class Animation extends JPanel implements Runnable
{
	protected Thread sim;      // animation thread
	protected int width;          // width of viewing area in pixels
	protected int height;         // height of viewing area in pixels
	protected int x, y;           // current position of text
	protected Dimension size;     // size of viewing area
	protected Image image;        // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field, robot;
	protected double scale; // pixels per meter
	protected int robotWidth, robotHeight;

	protected int dt;          // interval between frames in millisec
	private DrivetrainModel model;

	protected double time;

	public Animation() throws IOException {
		//    field = Toolkit.getDefaultToolkit().getImage("field.png");
		field = ImageIO.read(new File("field.png"));
		robot = ImageIO.read(new File("robot-red.png"));


		// Set the width and heigth and size
		width = field.getWidth(this);
		height = field.getHeight(this)/2;
		robotWidth = robot.getWidth(this);
		robotHeight = robot.getHeight(this);

		setSize ( width, height );
		scale = 1685.0/16.46;

		time = 0;

		// Set the time interval between frames in millisec
		dt = 10;

		// Set the initial values for x and y
		x = width / 2;
		y = height / 2;
		y = height;

		model = new DrivetrainModel(x/scale, 80.0/scale/2, 0.0);


		// Create and start the thread
		sim = new Thread ( this );
		sim.start();
	}


	// Update function
	public void paintComponent (Graphics g) {  
		double simTime = dt*1.0/1.0/1000.0;
		time += simTime;
		//	  System.out.println(time);

		double leftVoltage;
		double rightVoltage;
//		if (time < 0.3) {
//			rightVoltage = 0.0;
//			leftVoltage = 0.0;
//		} else if (time < 0.6) {
//			rightVoltage = 12.0;
//			leftVoltage = 5.0;
//		} else if (time < 1.2) {
//			rightVoltage = 6.0;
//			leftVoltage = 6.0;
//		} else if (time < 1.9) {
//			rightVoltage = 3.35;
//			leftVoltage = 9.65;
//		} else {
//			rightVoltage = 0.0;
//			leftVoltage = 0.0;
//		}
		
		if (time < 2.0) {
			rightVoltage = 12.0;
			leftVoltage = 12.0;
		} else {
			rightVoltage = 0.0;
			leftVoltage = 0.0;
		}

		//	  leftVoltage = 12.0;
		//	  rightVoltage = 12.0;

		//		CartesianCoordinate goal = new CartesianCoordinate((x/scale) + 4.0, (80.0/scale/2) + 4.0, -135.0);
		//		Tuple out = Drivetrain.driveToPoint(goal, model.center);
		//		leftVoltage = out.left*2;
		//		rightVoltage = out.right*2;

		model.update(leftVoltage, rightVoltage, simTime);

		// Get the size of the viewing area
		size = this.getSize();
		// Create the off-screen image buffer if it is the first time
		if ( image == null ) {
			image = createImage ( size.width, size.height );
			offScreen = image.getGraphics();
		}

		// Draw background field
		offScreen.drawImage(field, 0, -height, this);

		// Draw robot
		double xPixels = model.center.x * scale;
		double yPixels = model.center.y * scale;
		int xCoord = (int) Math.round(xPixels)-(robotWidth/2);
		int yCoord = y-(int) Math.round(yPixels)-(robotHeight/2);
		AffineTransform tx = AffineTransform.getRotateInstance(model.center.heading, robotWidth/2, robotHeight/2);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		offScreen.drawImage(op.filter((BufferedImage) robot, null), xCoord, yCoord, this);

		//		xPixels = goal.x * scale;
		//		yPixels = goal.y * scale;
		//		xCoord = (int) Math.round(xPixels);
		//		yCoord = y-(int) Math.round(yPixels);
		//		offScreen.setColor ( Color.yellow );
		//		offScreen.drawOval(xCoord, yCoord, 2, 2);

		// Copy the off-screen image to the screen
		g.drawImage ( image, 0, 0, this );     
	}

	@Override
	public void update ( Graphics g ) {
		paintComponent ( g );
	}

	@Override
	public void run () {
		while ( Thread.currentThread() == sim ) {
			repaint ();
			try {
				Thread.sleep ( dt );
			}
			catch (InterruptedException e) {
				System.out.println ("Exception: " + e.getMessage());
			}        
		}
	}

	public static void main ( String[] args ) throws IOException {
		JFrame frame = new JFrame ( "Drivetrain Simulation" );
		Animation panel = new Animation ();
		frame.getContentPane().add ( panel );
		frame.setSize ( panel.width, panel.height );
		frame.setVisible ( true );
		frame.addWindowListener (new WindowAdapter() {
			public void windowClosing ( WindowEvent evt ) {
				System.exit ( 0 );
			}
		});
	}
}