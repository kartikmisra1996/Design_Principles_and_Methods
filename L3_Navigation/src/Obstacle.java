
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.Sound;
import lejos.hardware.device.LMotor;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/*
 * The robot will navigate the course while avoiding obstacles
 * 
 * Authors: Kartik MIsra, Alexis Franche
 * 
 */

public class Obstacle implements UltrasonicController, Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odo;
	private static Navigate navigate;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor servoMotor;
	private static final double CHEAT_DISTANCE = 30;
	private static double TURN_ANGLE;
	private static final double TILE_SIZE = 30.48;
	private int distance;


	double wheel_radius = Lab3.WHEEL_RAD;
	double width = Lab3.TRACK;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;

	private static Lock lock = new ReentrantLock(true);
	private volatile boolean isReseting = false;
	private Condition doneReseting = lock.newCondition();

	public Odometer odometer;

	public Obstacle(Odometer odometer){ //constructor
		this.odometer = odometer;
	}

	public void run(){
		try {
			travelTo(1,1);
			travelTo(0,2);
			travelTo(2,2);
			travelTo(2,1);
			travelTo(1,0);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

	/*
	 * Author: Kartik Mira, ALexis Franche
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction. 
	 * Modification: added Timeunit delays at each rotation to give the sensor time to detect a potential wall
	 * 
	 * @param x and y values to travel to
	 * @void commands the robot to rotate and set the travel distance to reach destination
	 * @thrws InterruptedExcpetion for Timeunit delay
	 * 
	 */
	public void travelTo(double pointX, double pointY) throws InterruptedException{
		TimeUnit.SECONDS.sleep(1);
		this.curx = pointX;
		this.cury = pointY;
		double[] location = new double[3];
		location = odometer.getXYT();
		double actualTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];


		odo_x = actualX;
		odo_y = actualY;
		odo_theta = actualTheta;

		double finalX = pointX * TILE_SIZE;
		double finalY = pointY * TILE_SIZE;


		double yDiff = finalY-odo_y;
		double xDiff = finalX-odo_x;

		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));

		double distFInal = Math.hypot(xDiff,yDiff);

		double angleFinal = angleCalc - odo_theta;

		if(angleFinal < -180){ 
			turnTo(angleFinal + 360);
		}
		else if(angleFinal > 180){
			turnTo(angleFinal - 360);
		}
		else{
			turnTo(angleFinal);
		}
		
		TimeUnit.SECONDS.sleep(1);
		if (this.distance < 35){

			avoidRight((int)distance);

		}
		else {
			goTo(distFInal);
		}

	}
	
	/*Authors: ALexis Franche, Kartik Misra
	 * 
	 * Commands the robot to move exactly the input diatance
	 * 
	 * @param length of distance to travel when called given the odometer XYT and the final coordinates
	 * @void commands the robot to travel the input distance
	 * 
	 */
	public void goTo(double distance){
		Lab3.leftMotor.setSpeed(FORWARD_SPEED);
		Lab3.rightMotor.setSpeed(FORWARD_SPEED);

		Lab3.leftMotor.rotate(convertDistance(wheel_radius, distance), true);
		Lab3.rightMotor.rotate(convertDistance(wheel_radius, distance), false);
	}

	/*Authors: Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to rotate to the input distance
	 * 
	 * @param angle to turn to
	 * @void command the robot wheels to move to reach input distance
	 */
	public void turnTo(double theta){
		Lab3.leftMotor.setSpeed(ROTATE_SPEED);
		Lab3.rightMotor.setSpeed(ROTATE_SPEED);

		Lab3.leftMotor.rotate(convertAngle(wheel_radius, width, theta), true);
		Lab3.rightMotor.rotate(-convertAngle(wheel_radius, width, theta), false);
	}
	
	/*Author: Alexis Franche
	 * 
	 * Checks if the wheels are moving, if yes return true, false otherwise
	 * 
	 * @return 
	 * 
	 */
	public boolean isNavigating(){
		return (Lab3.leftMotor.isMoving() && Lab3.rightMotor.isMoving());
	}

	private static int convertDistance(double radius, double distance1) {
		return (int) ((180.0 * distance1) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public void avoidLeft(int distance) throws InterruptedException{

	}

	/*
	 * Authors: Kartik Misra, Alexis Franche
	 * 
	 * Depending on the distance of the robot from the wall, apply a more or less aggressive correction
	 * to bypass it then call goTo to resume the path and travel to the initial coordinates the robot
	 * was supposed to go to 
	 * 
	 * @param distance from US sensor
	 * @void apply course correction
	 * @throws InterruptedException for Timeunit delay
	 */
	public void avoidRight(int distance) throws InterruptedException {
		if ((this.distance > 0) && (this.distance < 10)) {
			TURN_ANGLE = 90;
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 23), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 23), false);
			Lab3.leftMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 23), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 23), false);
		}

		else if (this.distance >10 && this.distance < 20) {
			TURN_ANGLE = 73;
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 23), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 23), false);
			Lab3.leftMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 23), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 23), false);
		}

		else if (this.distance > 20 && this.distance < 37) {
			TURN_ANGLE = 50;
			Lab3.leftMotor.setSpeed(ROTATE_SPEED);
			Lab3.rightMotor.setSpeed(ROTATE_SPEED);
			Lab3.leftMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 30), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 30), false);
			Lab3.leftMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), true);
			Lab3.rightMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), false);
			Lab3.leftMotor.rotate(convertDistance(2.1, 27), true);
			Lab3.rightMotor.rotate(convertDistance(2.1, 27), false);
		}

		TimeUnit.SECONDS.sleep(2);

		if (this.distance < 37) {
			avoidRight(distance);
		}
		else {
			travelTo(curx, cury);
		}
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
