package frc.simulation;

import javax.imageio.ImageIO;
import javax.swing.*;

import frc.controls.PathFollower;
import frc.controls.Waypoint;
import frc.controls.WaypointNavigator;
import frc.robot.subsystems.DrivetrainModel;
import frc.util.Pose;
import frc.util.Tuple;

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
	protected Dimension size;     // size of viewing area
	protected Image image;        // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field, robot;
	protected final double speed = 1.0;
	protected final double scale = 65; // pixels per meter
	protected static final int offsetX = 5;
	protected static final int offsetY = 15; 
	protected final int x;
	protected final int y;
	protected int robotWidth, robotHeight;

	protected int dt = 20;          // interval between frames in millisec
	private DrivetrainModel model;
	private frc.controls.WaypointNavigator waypointNav;

	protected double time = 0.0;

	public Animation() throws IOException {
		field = ImageIO.read(new File("field-blue.png"));
		robot = ImageIO.read(new File("robot-blue.png"));

		// Set the width and heigth and size
		width = field.getWidth(this);
		height = field.getHeight(this);
		robotWidth = robot.getWidth(this);
		robotHeight = robot.getHeight(this);
		setSize(width, height);	
		x = 0 + offsetX;	
		y = height-offsetY;
		
		model = new DrivetrainModel();
		waypointNav = new WaypointNavigator();

		// // start pos
		// model.setPosition(2.85, 1.7, 0.0);
		// waypointNav.addWaypoint(new Waypoint(2.8, 1.7, 0.0));
		// waypointNav.addWaypoint(new Waypoint(2.8, 3.5, 0.0, 0.0, 0.5, 0.5, false));
		
		// // Left to side of cargo ship
		// model.setPosition(2.85, 1.7, 0.0);
		// waypointNav.addWaypoint(new Waypoint(2.85, 1.7, 0.0));
		// waypointNav.addWaypoint(new Waypoint(2.85, 3.5, 0.0, 0.3, 0.5, 0.5, false));
		// waypointNav.addWaypoint(new Waypoint(2.9, 6.7, 0.0, 1.0, 1.2, 0.4, false));


		// Left to rocket ship far
		model.setPosition(2.85, 1.7, 0.0);
		waypointNav.addWaypoint(new Waypoint(2.85, 1.7, 0.0));
		waypointNav.addWaypoint(new Waypoint(2.85, 3.5, 0.0, 0.3, 0.5, 0.5, false));
		waypointNav.addWaypoint(new Waypoint(1.2, 6.4, 0.0, 1.0, 1.2, 0.4, false));
		waypointNav.addWaypoint(new Waypoint(0.3, 6.8, 0.0, 0.2, 0.5, 0.5, false));

		// // Left to  rocket ship close OLD
		// model.setPosition(3.0, 1.7, 0.0);
		// waypointNav.addWaypoint(new Waypoint(3.0, 1.7, 0.0));
		// waypointNav.addWaypoint(new Waypoint(3.0, 3.5, 0.0, 0.6, 0.5));
		// waypointNav.addWaypoint(new Waypoint(0.5, 4.4, 0.0, 0.4, 0.7));
		// waypointNav.addWaypoint(new Waypoint(0.5, 4.7, 0.0, 0.4, 0.4));
		// waypointNav.addWaypoint(new Waypoint(0.7, 5.0, 0.0, 0.5, 0.4));

		// // Cargo ship to loading zone
		// model.setPosition(2.9, 6.7, 0.0);
		// waypointNav.addWaypoint(new Waypoint(2.9, 6.7, 0.0));
		// waypointNav.addWaypoint(new Waypoint(2.9, 5.0, 0.0, 0.7, 0.9, 0.9, true));
		// waypointNav.addWaypoint(new Waypoint(0.6, 1.0, 0.0, 1.0, 1.3, 0.4, true));
		// waypointNav.addWaypoint(new Waypoint(0.6, 0.4, 0.0, 0.2, 0.4, 0.3, true));

		// // Rocket far to loading zone
		// model.setPosition(0.7, 6.7, -60.0);
		// waypointNav.addWaypoint(new Waypoint(0.7, 6.7, 0.0));
		// waypointNav.addWaypoint(new Waypoint(1.5, 5.5, 0.0, 0.7, 0.9, 0.9, true));
		// waypointNav.addWaypoint(new Waypoint(0.6, 1.0, 0.0, 1.0, 1.3, 0.4, true));
		// waypointNav.addWaypoint(new Waypoint(0.6, 0.4, 0.0, 0.2, 0.4, 0.3, true));

		// // Loading zone to rocket close 
		// model.setPosition(0.6, 0.4, 0.0);
		// waypointNav.addWaypoint(new Waypoint(0.6, 0.4, 0.0));
		// waypointNav.addWaypoint(new Waypoint(0.5, 4.9, 0.0, 0.1, 0.7));
		// waypointNav.addWaypoint(new Waypoint(0.7, 5.1, 0.0, 0.5, 0.5));

			
		// Create and start the thread
		sim = new Thread ( this );
		sim.start();
	}


	// Update function
	public void paintComponent (Graphics g) {  
		double simTime = dt*1.0/1000.0/speed;
		time += simTime;

		Waypoint goal = waypointNav.updatePursuit(model.center);
		Tuple out = PathFollower.driveToPoint2(goal, model.center);
		Tuple limitedVoltage = model.limitAcceleration(out);
		// out = Drivetrain.driveToPoint(goal, model.center);
		// out = Drivetrain.deadReckoningCurveLeft(time);
		// out = Drivetrain.deadReckoningStraight(time);

		double leftVoltage = limitedVoltage.left*12.0;
		double rightVoltage = limitedVoltage.right*12.0;
		
		System.out.println("Left Voltage: " + leftVoltage + ", Right Voltage: " + rightVoltage);
		
		// leftVoltage = 4.0;
		// rightVoltage = 3.0;
		model.updateVoltage(leftVoltage, rightVoltage, simTime);
		model.updatePosition(simTime);

		// Get the size of the viewing area
		size = this.getSize();
		// Create the off-screen image buffer if it is the first time
		if ( image == null ) {
			image = createImage ( size.width, size.height);
			offScreen = image.getGraphics();
		}

		// Draw background field
		offScreen.drawImage(field, 0, 0, this);

		// Draw robot
		int xCoord = x + (int) Math.round(model.center.x * scale)-(robotWidth/2);
		int yCoord = y - (int) Math.round(model.center.y * scale)-(robotHeight/2);
		AffineTransform tx = AffineTransform.getRotateInstance(model.center.r, robotWidth/2, robotHeight/2);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		offScreen.drawImage(op.filter((BufferedImage) robot, null), xCoord, yCoord, this);

		// Draw Pursuit Point
		xCoord = x + (int) Math.round(goal.x * scale);
		yCoord = y - (int) Math.round(goal.y * scale);
		offScreen.setColor ( Color.yellow );
		offScreen.drawOval(xCoord, yCoord, 6, 6);

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
			} catch (InterruptedException e) {
				System.out.println ("Exception: " + e.getMessage());
			}        
		}
	}

	public static void main ( String[] args ) throws IOException {
		JFrame frame = new JFrame ( "Drivetrain Simulation" );
		Animation panel = new Animation ();
		frame.getContentPane().add ( panel );
		frame.setSize ( panel.width, panel.height  + 35);
		frame.setVisible ( true );
		frame.addWindowListener (new WindowAdapter() {
			public void windowClosing ( WindowEvent evt ) {
				System.exit ( 0 );
			}
		});
	}
}