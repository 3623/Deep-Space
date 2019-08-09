package frc.modeling;

import javax.imageio.ImageIO;
import javax.swing.*;

import frc.controls.CubicSplineFollower;
import frc.controls.DrivetrainControls;
import frc.controls.CubicSplineFollower.Waypoint;
import frc.robot.subsystems.DrivetrainModel;
import frc.util.Pose;
import frc.util.Tuple;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class Animation extends JPanel implements Runnable {

	public static void main(String[] args) throws IOException {
		JFrame frame = new JFrame("Drivetrain Simulation");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		Container pane = frame.getContentPane();
		Animation panel = new Animation();
		pane.add(panel, BorderLayout.CENTER);

		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}

	protected Thread sim; // animation thread
	protected int width; // width of viewing area in pixels
	protected int height; // height of viewing area in pixels
	protected Dimension size; // size of viewing area
	protected Image image; // off-screen image
	protected Graphics offScreen; // off-screen graphics
	protected Image field;
	protected BufferedImage robot;
	protected final double scale = 172.4; // pixels per meter
	protected final int x;
	protected final int y;
	protected int robotWidth, robotHeight;
	protected ArrayList<Tuple> trajectory;// array for storing points passed through by robot

	protected final int SPEED = 1; // replay speed
	protected final int FRAME_RATE = 60; // interval between frames in millisec

	private DrivetrainModel model;
	private CubicSplineFollower nav;

	protected final int ODOMETRY_UPDATE_RATE = 200;
	protected final int CONTROL_UPDATE_RATE = 200;
	protected double leftVoltage = 0.0;
	protected double rightVoltage = 0.0;

	protected double time = 0.0;

	public Animation() throws IOException {
		field = ImageIO.read(new File("IWIC/2019-field-blue.png"));
		robot = ImageIO.read(new File("IWIC/robot-blue2.png"));

		// Set the width and heigth and size
		width = field.getWidth(this);
		height = field.getHeight(this);
		robotWidth = robot.getWidth(this);
		robotHeight = robot.getHeight(this);

		x = 0 + width / 2; // the x offset for drawing objects
		y = height - 15; // y offset for drawing objects

		model = new DrivetrainModel();
		nav = new CubicSplineFollower();

		this.setWaypoints();

		sim = new Thread(this); // Create and start the thread
		sim.start();
		odometryThread();
		controlThread();

		trajectory = new ArrayList<Tuple>();
	}

	private void setWaypoints() {

		// // Spider Y 2 Banana
		// model.setPosition(0.5, 0.5, 0.0);
		// nav.addWaypoint(new Waypoint(1.1, 3.0, 25.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(0.0, 1.0, 0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(0.0, 2.0, 0.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-0.3, 1.3, 45.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(1.1, 3.0, 30.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(0.0, 1.0, 0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(0.0, 2.5, 0.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-0.3, 1.5, 45.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(1.1, 3.0, 30.0, 1.0, true));

		// // Poofs 2018 Near Side
		// model.setPosition(2.8, 0.5, 0.0);
		// nav.addWaypoint(new Waypoint(2.2, 7.3, -10.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(1.9, 5.5, 30.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(2.2, 7.3, -10.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(1.5, 5.7, 40.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(2.2, 7.3, 0.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(1.0, 5.6, 60.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(2.2, 7.3, 0.0, 1.0, true));

		// // Poofs 2018 Oppos Side
		// model.setPosition(2.8, 0.5, 0.0);
		// nav.addWaypoint(new Waypoint(2.2, 5.6, -40.0, 1.0));
		// nav.addWaypoint(new Waypoint(-1.7, 6.0, -80.0, 0.8));
		// nav.addWaypoint(new Waypoint(-2.2, 7.3, 15.0, 0.6, true));
		// nav.addWaypoint(new Waypoint(-2.2, 5.8, -25.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-2.2, 7.3, 15.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-1.5, 5.7, -40.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-2.2, 7.3, 15.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-1.0, 5.6, -65.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-2.2, 7.3, 15.0, 1.0, true));

		// // Right Side of cargo ship x2
		// model.setPosition(1.2, 0.7, 0.0);
		// nav.addWaypoint(new Waypoint(1.2, 3.0, 0.0, 0.5, false));
		// nav.addWaypoint(new Waypoint(1.1, 6.6, -10.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(3.3, 0.5, -10.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(1.1, 7.2, -10.0, 1.0, true));

		// // Left Side of cargo ship x2
		// model.setPosition(-1.2, 0.7, -0.0);
		// nav.addWaypoint(new Waypoint(-1.2, 3.0, -0.0, 0.5, false));
		// nav.addWaypoint(new Waypoint(-1.1, 6.6, 10.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-3.3, 0.5, 10.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-1.1, 7.2, 10.0, 1.0, true));

		// Right rocket
		model.setPosition(1.2, 0.7, 0.0);
		nav.addWaypoint(new Waypoint(1.2, 3.0, 0.0, 0.5));
		nav.addWaypoint(new Waypoint(3.5, 6.7, 60.0, 1.0, true));
		nav.addWaypoint(new Waypoint(2.8, 6.0, 30.0, -0.6));
		nav.addWaypoint(new Waypoint(3.4, 0.5, 0.0, -1.0, true));
		nav.addWaypoint(new Waypoint(3.4, 3.0, 0.0, 1.0));
		nav.addWaypoint(new Waypoint(3.2, 4.7, -60.0, 0.6, true));
		nav.addWaypoint(new Waypoint(2.0, 7.2, 0.0, 0.6, true));

		// // Left rocket
		// model.setPosition(-1.2, 0.7, -0.0);
		// nav.addWaypoint(new Waypoint(-1.2, 3.0, -0.0, 0.5));
		// nav.addWaypoint(new Waypoint(-3.5, 6.6, -60.0, 1.0, true));
		// nav.addWaypoint(new Waypoint(-2.8, 6.1, -50.0, -0.6));
		// nav.addWaypoint(new Waypoint(-3.4, 0.5, -0.0, -1.0, true));
		// nav.addWaypoint(new Waypoint(-3.4, 4.3, -0.0, 1.0));
		// nav.addWaypoint(new Waypoint(-3.2, 4.7, 60.0, 0.6, true));

		// // Beautiful 90
		// model.setPosition(0.0, 1.0, 0.0);
		// nav.addWaypoint(new Waypoint(1.0, 3.0, 90.0, 1.0, true));

		// // Tine S Curve
		// model.setPosition(0.0, 1.0, 0.0);
		// nav.addWaypoint(new Waypoint(0.3, 2.0, 0.0, 0.5, true));
	}

	// Update function
	public void paintComponent(Graphics g) {
		super.paintComponent(g);

		size = this.getSize(); // Get the size of the viewing area
		if (image == null) { // Create the off-screen image buffer if it is the first time
			image = createImage(size.width, size.height);
			offScreen = image.getGraphics();
		}

		offScreen.drawImage(field, 0, 0, this); // Draw background field

		/// Draw robot
		BufferedImage robotRotated = rotateRobot();
		int xCoord = x + (int) Math.round(model.center.x * scale);
		int xCoordOffset = xCoord - (robotRotated.getWidth() / 2);
		int yCoord = y - (int) Math.round(model.center.y * scale);
		int yCoordOffset = yCoord - (robotRotated.getHeight() / 2);
		if (!nav.getIsFinished())
			trajectory.add(new Tuple(xCoord, yCoord));
		offScreen.drawImage(robotRotated, xCoordOffset, yCoordOffset, this);

		// Draw waypoints
		int waypointX = x + (int) Math.round(nav.getCurrentWaypoint().x * scale);
		int waypointY = y - (int) Math.round(nav.getCurrentWaypoint().y * scale);
		offScreen.setColor(Color.yellow);
		offScreen.drawOval(waypointX - 3, waypointY - 3, 6, 6);

		// Draw trajectory
		for (Tuple point : trajectory) {
			offScreen.setColor(Color.yellow);
			offScreen.drawOval((int) point.left, (int) point.right, 1, 1);
		}

		// Copy the off-screen image to the screen and scale to fit jframe
		Graphics2D g2 = (Graphics2D) g;
		g2.scale(size.width / (double) this.width, size.height / (double) this.height);
		g2.drawImage(image, 0, 0, this);
	}

	public BufferedImage rotateRobot() {
		double sin = Math.abs(Math.sin(model.center.r));
		double cos = Math.abs(Math.cos(model.center.r));

		int newWidth = (int) Math.floor(robotWidth * cos + robotHeight * sin);
		int newHeight = (int) Math.floor(robotHeight * cos + robotWidth * sin);

		BufferedImage rotated = new BufferedImage(newWidth, newHeight, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2d = rotated.createGraphics();
		AffineTransform at = new AffineTransform();
		at.translate((newWidth - robotWidth) / 2, (newHeight - robotHeight) / 2);

		int x = robotWidth / 2;
		int y = robotHeight / 2;

		at.rotate(model.center.r, x, y);
		g2d.setTransform(at);
		g2d.drawImage(robot, 0, 0, this);
		g2d.dispose();

		return rotated;
	}

	@Override
	public Dimension getPreferredSize() {
		return new Dimension(width, height);
	}

	@Override
	public Dimension getMinimumSize() {
		return new Dimension(width / 2, height / 2);
	}

	@Override
	public Dimension getMaximumSize() {
		return new Dimension(width * 3, height * 3);
	}

	@Override
	public void run() {
		while (Thread.currentThread() == sim && nav.getIsFinished() == false) {
			repaint();
			try {
				Thread.sleep(1000 / FRAME_RATE);
			} catch (InterruptedException e) {
				System.out.println("Exception: " + e.getMessage());
			}
		}
	}

	public void controlThread() {
		Thread c = new Thread(() -> {
			while (!Thread.interrupted()) {

				Tuple output = nav.updatePursuit(model.center);
				output = model.limitAcceleration(output);
				leftVoltage = output.left * 12.0;
				rightVoltage = output.right * 12.0;

				// System.out.println("Left Voltage: " + leftVoltage + ", Right Voltage: " +
				// rightVoltage);

				if (nav.getIsFinished()) {
				}

				try {
					Thread.sleep(1000 / CONTROL_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		c.start();
	}

	public void odometryThread() {
		Thread o = new Thread(() -> {
			while (!Thread.interrupted()) {
				time += 1000 / ODOMETRY_UPDATE_RATE;

				// leftVoltage += (Math.random() - 0.5) * 4.0; // Some random error. Well, a lot
				// rightVoltage += (Math.random() - 0.5) * 4.0;

				model.updateVoltage(leftVoltage, rightVoltage, 1.0 / ODOMETRY_UPDATE_RATE);
				model.updatePosition(1.0 / ODOMETRY_UPDATE_RATE);
				// System.out.println(model.center.x);

				try {
					Thread.sleep(1000 / ODOMETRY_UPDATE_RATE * SPEED);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		o.start();
	}
}