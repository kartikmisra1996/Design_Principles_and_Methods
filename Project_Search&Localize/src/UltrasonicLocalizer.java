

import java.awt.Button;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.*;

/*
 * The robot will approximate location of 0 degree point using the
 * ultrasonic sensor
 * 
 * Authors: Kartik MIsra, Alexis Franche
 */

public class UltrasonicLocalizer implements UltrasonicController, Runnable {

	public enum LocType {FALLING_EDGE, RISING_EDGE}
	public LocType locType;

	private static double TURN_ANGLE = 90;
	private int distance;

	private double firstAngleFE, secondAngleFE, firstAngleRE, secondAngleRE;

	double wheel_radius = Lab5.WHEEL_RAD;
	double width = Lab5.TRACK;
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 50;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;

	public Odometer odometer;

	public UltrasonicLocalizer(Odometer odometer, LocType locType){ //constructor
		this.odometer = odometer;
		this.locType = locType;
	}

	public void run(){
		if (this.locType == LocType.FALLING_EDGE) {
			fallingEdge(distance);
		}
		else if (this.locType == LocType.RISING_EDGE) {
			risingEgde(distance);
		}
	}

	/* Author: Alexis Franche
	 * 
	 * The robot will rotate and scan the falling edge of the wall to figure our where it is relative
	 * to the (0,0) point. Falling edge will switch directions when it detects a wall
	 * 
	 * @param input distance from constructor
	 * @void 
	 */
	public void fallingEdge (int distance) {
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);
		Sound.beep();

		//rotate until can't detect wall
		while (this.distance <= 40) {
			Lab5.leftMotor.forward();
			Lab5.rightMotor.backward();
		}

		//rotate until it sees a wall
		while (this.distance > 40) {
			//Sound.beep();
			Lab5.leftMotor.forward();
			Lab5.rightMotor.backward();
		}

		//get angle
		firstAngleFE = odometer.getTheta();

		//rotate until can't see wall
		while (this.distance <= 40) {
			//Sound.beepSequence();
			Lab5.leftMotor.backward();
			Lab5.rightMotor.forward();
		}

		//rotate until can see wall
		while (this.distance > 40) {
			//Sound.beepSequence();
			Lab5.leftMotor.backward();
			Lab5.rightMotor.forward();
		}

		//get angle
		secondAngleFE = odometer.getTheta();

		double avgAngleFE = (firstAngleFE + secondAngleFE)/2;
		double zeroPointFE =  secondAngleFE - avgAngleFE - 45;

		turnTo(zeroPointFE+30);

		odometer.setXYT(0, 0, 0);

		Lab5.leftMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), true);
		Lab5.rightMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), false);
		Lab5.leftMotor.rotate(convertDistance(2.1, 6), true);
		Lab5.rightMotor.rotate(convertDistance(2.1, 6), false);
		Lab5.leftMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), true);
		Lab5.rightMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), false);
		Lab5.leftMotor.rotate(convertDistance(2.1, 6), true);
		Lab5.rightMotor.rotate(convertDistance(2.1, 6), false);

		odometer.setXYT(0, 0, 0);

		Lab5.leftMotor.stop();
		Lab5.rightMotor.stop();
	}

	/* Author: Alexis Franche
	 * 
	 * The robot will rotate and scan the rising edge of the wall to figure our where it is relative
	 * to the (0,0) point. Rising edge will switch directions when it detects no wall
	 * 
	 * @param input distance from constructor
	 * @void 
	 */
	public void risingEgde(int distance) {
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);
		Sound.beep();

		//rotate until can't detect wall
		while (this.distance >= 40) {
			Lab5.leftMotor.forward();
			Lab5.rightMotor.backward();
			System.out.println("          " + this.distance);
		}

		//rotate until it sees a wall
		while (this.distance < 40) {
			Lab5.leftMotor.forward();
			Lab5.rightMotor.backward();
			System.out.println("          " + this.distance);
		}

		//get angle
		firstAngleRE = odometer.getTheta();

		//rotate until can't see wall
		while (this.distance >= 40) {
			Lab5.leftMotor.backward();
			Lab5.rightMotor.forward();
			System.out.println("          " + this.distance);
		}

		//rotate until can see wall
		while (this.distance < 40) {
			Lab5.leftMotor.backward();
			Lab5.rightMotor.forward();
			System.out.println("          " + this.distance);
		}

		//get angle
		secondAngleRE = odometer.getTheta();

		if(firstAngleRE > firstAngleRE){
			firstAngleRE = firstAngleRE - 360;
		}

		double avgAngleRE = (firstAngleRE + secondAngleRE)/2;
		double zeroPointRE =  secondAngleRE - avgAngleRE + 45;

		turnTo(zeroPointRE + 80);

		odometer.setXYT(0, 0, 0);

		Lab5.leftMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), true);
		Lab5.rightMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), false);
		Lab5.leftMotor.rotate(convertDistance(2.1, 6), true);
		Lab5.rightMotor.rotate(convertDistance(2.1, 6), false);
		Lab5.leftMotor.rotate(-convertAngle(2.1, 13.6, TURN_ANGLE), true);
		Lab5.rightMotor.rotate(convertAngle(2.1, 13.6, TURN_ANGLE), false);
		Lab5.leftMotor.rotate(convertDistance(2.1, 6), true);
		Lab5.rightMotor.rotate(convertDistance(2.1, 6), false);

		odometer.setXYT(0, 0, 0);

		Lab5.leftMotor.stop();
		Lab5.rightMotor.stop();
	}

	/*Authors: Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to rotate to the input distance
	 * 
	 * @param angle to turn to
	 * @void command the robot wheels to move to reach input distance
	 */
	public void turnTo(double theta){
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

		Lab5.leftMotor.rotate(convertAngle(wheel_radius, width, theta), true);
		Lab5.rightMotor.rotate(-convertAngle(wheel_radius, width, theta), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
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
