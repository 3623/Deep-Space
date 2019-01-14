package frc.simulation;

import javax.imageio.ImageIO;
import javax.swing.*;

import frc.controls.PathFollower;
import frc.controls.Waypoint;
import frc.controls.WaypointNavigator;
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
	protected int x, y;           // current position of text
	protected Dimension size;     // size of viewing area
	protected Image image;        // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field, robot;
	protected double speed;
	protected double scale, offset; // pixels per meter
	protected int robotWidth, robotHeight;

	protected int dt;          // interval between frames in millisec
	private DrivetrainModel model;
	private frc.controls.WaypointNavigator waypointNav;

	protected double time;

	public Animation() throws IOException {
		//    field = Toolkit.getDefaultToolkit().getImage("field.png");
		field = ImageIO.read(new File("field-blue.png"));
		robot = ImageIO.read(new File("robot-blue.png"));


		// Set the width and heigth and size
		width = field.getWidth(this);
		height = field.getHeight(this);
		robotWidth = robot.getWidth(this);
		robotHeight = robot.getHeight(this);
		
		speed = 1.0;

		setSize ( width, height );
		scale = 101;
		offset = 10.0;

		time = 0;

		// Set the time interval between frames in millisec
		dt = 10;

		// Set the initial values for x and y
		x = width / 2;
		y = height;
		
		// Poofs Iconic Gear then Shoot
		model = new DrivetrainModel(2.2, 0.4, 0.0);
		waypointNav = new WaypointNavigator();
		waypointNav.addWaypoint(new Waypoint(2.2, 0.0, -90.0));
		waypointNav.addWaypoint(new Waypoint(2.2, 2.5, -90.0, 0.6, 0.7));
		waypointNav.addWaypoint(new Waypoint(3, 3.1, -90.0, 0.2, 0.6));
		waypointNav.addWaypoint(new Waypoint(0.3, 4.1, -90.0, 1.2, 0.7));
		waypointNav.addWaypoint(new Waypoint(0.4, 2.8, -90.0, 0.75, 0.6));
		
//		 Poofs Iconic 2018 auto
		model = new DrivetrainModel(1.9, 0.4, 0.0);
		waypointNav = new WaypointNavigator();
		waypointNav.addWaypoint(new Waypoint(1.3, 0.0, -90.0));
		waypointNav.addWaypoint(new Waypoint(1.3, 5.3, -90.0, 0.7, 0.7));
		waypointNav.addWaypoint(new Waypoint(7, 5.3, -90.0, 0.6, 0.7));
		waypointNav.addWaypoint(new Waypoint(7, 5.9, -90.0, 0.6, 0.4));


		
//		// S Curve that we would have used if we had an actual drivetrain in 2018
//		model = new DrivetrainModel(4.2, 0.4, 0.0);
//		waypointNav = new WaypointNavigator();
//		waypointNav.addWaypoint(new Waypoint(4.2, 0.4, 0.0));
//		waypointNav.addWaypoint(new Waypoint(6., 2.3, 0.0, 0.4, 0.6));
//		waypointNav.addWaypoint(new Waypoint(5.94, 2.6, 0.0, 0.3, 0.6));
		
//		// Square with initial correction
//		model = new DrivetrainModel(6.5, 0.4, 0.0);
//		waypointNav = new WaypointNavigator();
//		waypointNav.addWaypoint(new Waypoint(7.0, 0.4, 0.0));
//		waypointNav.addWaypoint(new Waypoint(7.0, 5.4, 0.0, 0.6, 0.7));
//		waypointNav.addWaypoint(new Waypoint(1.0, 5.4, 0.0, 0.1, 0.7));
//		waypointNav.addWaypoint(new Waypoint(1.0, 0.45, 0.0, 0.6, 0.7));
//		waypointNav.addWaypoint(new Waypoint(6.0, 0.45, 0.0, 0.6, 0.7));


		// Create and start the thread
		sim = new Thread ( this );
		sim.start();
	}


	// Update function
	public void paintComponent (Graphics g) {  
		double simTime = dt*1.0/1000.0/speed;
		time += simTime;


		Tuple out;
		Pose goal = waypointNav.updatePursuit(model.center);
		out = PathFollower.driveToPoint(goal, model.center);

//		out = Drivetrain.driveToPoint(goal, model.center);
//		out = Drivetrain.deadReckoningCurveLeft(time);
//		out = Drivetrain.deadReckoningStraight(time);
		double leftVoltage = out.left*10;
		double rightVoltage = out.right*10;
//		leftVoltage = 12.0;
//		rightVoltage = 0.0;
		model.update(leftVoltage, rightVoltage, simTime);

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
		double xPixels = model.center.x * scale + offset;
		double yPixels = model.center.y * scale + offset;
		int xCoord = (int) Math.round(xPixels)-(robotWidth/2);
		int yCoord = y-(int) Math.round(yPixels)-(robotHeight/2);
		AffineTransform tx = AffineTransform.getRotateInstance(model.center.heading, robotWidth/2, robotHeight/2);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
		offScreen.drawImage(op.filter((BufferedImage) robot, null), xCoord, yCoord, this);

//		goal = new Pose(4.0, 4.0, -135.0);

		xPixels = goal.x * scale + offset;
		yPixels = goal.y * scale + offset;
		xCoord = (int) Math.round(xPixels);
		yCoord = y-(int) Math.round(yPixels);
		offScreen.setColor ( Color.yellow );
		offScreen.drawOval(xCoord, yCoord, 4, 4);

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
		frame.setSize ( panel.width, panel.height  + 35);
		frame.setVisible ( true );
		frame.addWindowListener (new WindowAdapter() {
			public void windowClosing ( WindowEvent evt ) {
				System.exit ( 0 );
			}
		});
	}
}