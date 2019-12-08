

import java.util.concurrent.TimeUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
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

/**
 * Search for the target block 
 * 
 * @author 
 *		 	Team 03
 * 			Kartik Misra
 * 			Alexis Franche
 * 			Wiam El Ouadi
 * 			Yuliya Volodina
 * 			Saifullah Ahmed
 * 			Jude Habib
 * 
 */

public class Searching implements UltrasonicController, Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odo;
	private static Navigate navigate;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor servoMotor;
	private static final double CHEAT_DISTANCE = 30;
	private static double TURN_ANGLE;
	private static final double TILE_SIZE = 30.48;
	private int distance ;

	public double LLX = 2;
	public double LLY = 2;
	public double URX = 5;
	public double URY = 5;
	double dimension = URY - LLY;
	//double dimensionX = URX - LLX;
	public double position[] = new double[3];

	public ArrayList<Double> dist2 = new ArrayList<Double>();
	public ArrayList<Double> dist1 = new ArrayList<Double>();
	public ArrayList<Double> angle1 = new ArrayList<Double>();
	public ArrayList<Double> angle2 = new ArrayList<Double>();


	double wheel_radius = Lab5.WHEEL_RAD;
	double width = Lab5.TRACK;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 75;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;

	private static Lock lock = new ReentrantLock(true);
	private volatile boolean isReseting = false;
	private Condition doneReseting = lock.newCondition();

	public Odometer odometer;

	public Searching(Odometer odometer){ //constructor
		this.odometer = odometer;
	}
	

	public void run(){
		try {
			ghettoSquareDriver2();
		}
		catch(InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * travel path around search zone and scan objects until target block is found
	 * 
	 * @throws InterruptedException
	 */
	
	public void ghettoSquareDriver2() throws InterruptedException {
		
		if (!(odometer.getXYT()[0] == LLX && odometer.getXYT()[1] == LLY)) {
			travelTo(LLX, LLY);
		}
		
		travelTo(LLX, LLY+dimension);
		firstCorner();			
		goToBlock1();
		/*travelTo(LLX+dimensionX, LLY);
		oppositeCorner();
		goToBlock2();*/		
		Lab5.rightMotor.stop();
		Lab5.leftMotor.stop();
	}
	
	/***
	 * travels to the corner where the search algorithm starts 
	 */
	
	public void firstCorner() {
		travelTo(LLX+dimension, LLY+dimension);
		turnTo(90);
		scanObject();
	}
	
	public void oppositeCorner() {
		travelTo(LLX, LLY);
		turnTo(90);//face along the line
		scanObject2();
	}
	
	/**
	 * When object detected during the 90 degrees turn, arraylists for angle and distance are implemented
	 */
	
	public void scanObject() {
		for(int k=10; k<91 ; k+=10) {
			turnTo((10));
			if (this.distance < dimension*20) {
				double temp = this.distance;
				double tempAngle = odometer.getTheta();
				dist1.add(new Double(temp));
				angle1.add(new Double(tempAngle));
			}
		}
	}
	
	public void scanObject2() {
		for(int k=10; k<91 ; k+=10) {
			turnTo((10));
			if (this.distance < dimension*20) {
				double temp = this.distance;
				double tempAngle = odometer.getTheta();
				dist2.add(new Double(temp));
				angle2.add(new Double(tempAngle));
			}
		}
	}
	
	/**
	 * travels the distance recorded in the scanObject method at the right angle
	 * 
	 */
	
	public void goToBlock1() {
		if (angle1.size() > 0 && dist1.size()>0) {
			for(int k = 0; k<angle1.size(); k++) {
				turnTo(angle1.get(k).doubleValue() - odometer.getTheta());
				goTo(dist1.get(k).doubleValue());
				if(this.distance < 6) {
					turnTo(180);
				}
				if (ColorID.yellowDetected()) {
					travelTo(URX,URY);
					Lab5.rightMotor.stop();
					Lab5.leftMotor.stop();
					break;
				}
				else {
					travelTo(URX,URY);
					continue;
				}
			}
		} 
	}

	public void goToBlock2() {
		if (angle2.size() > 0 && dist2.size()>0) {
			for(int k = 0; k<angle2.size(); k++) {
				turnTo(angle2.get(k).doubleValue()- odometer.getTheta());
				goTo(dist2.get(k).doubleValue());
				if(this.distance < 6) { //so that the robot doesnt crash into the wall
					turnTo(180);
				}
				if (ColorID.yellowDetected()) {
					travelTo(LLX,LLY);
					travelTo(URX,URY);
					Lab5.rightMotor.stop();
					Lab5.leftMotor.stop();
					break;
				}
				else {
					travelTo(LLX,LLY);
					continue;
				}
			}
		} 
	}


	/**
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction. 
	 * @param x
	 * @param y
	 */

	public void travelTo(double x, double y) {

		x = x*TILE_SIZE;
		y = y*TILE_SIZE;

		position=odometer.getXYT();	
		double currX = position[0];
		double currY = position[1];	
		
		double distX = x - currX;
		double distY = y - currY;
		
		double destTheta = Math.toDegrees(Math.atan2(distX, distY));
		double distance = Math.sqrt( Math.pow(distX, 2) + Math.pow(distY, 2));

		turnTo(destTheta);

		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, (distance)), true);
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, (distance)), false);


		while(isNavigating()) {
			continue;
		}

	}

	/**
	 * Commands the robot to move exactly the input diatance
	 * @param distance
	 */
	
	public void goTo(double distance){
		Lab5.leftMotor.setSpeed(FORWARD_SPEED);
		Lab5.rightMotor.setSpeed(FORWARD_SPEED);

		Lab5.leftMotor.rotate(convertDistance(wheel_radius, distance), true);
		Lab5.rightMotor.rotate(convertDistance(wheel_radius, distance), false);
	}

	/**
	 * Commands the robot to rotate to the input distance.
	 * @param theta
	 */
	

	public void turnTo(double theta){
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

		double turnTheta = theta - position[2];
		if(turnTheta <-180) {
			turnTheta += 360;
		}
				
		else if(turnTheta>180) {
			turnTheta-=360;
		}
			
		Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD , Lab5.TRACK, turnTheta), true);
		Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK,turnTheta), false);

	}

	/**
	 * Checks if the wheels are moving, if yes return true, false otherwise
	 * @return
	 */
	
	public boolean isNavigating(){
		return (Lab5.leftMotor.isMoving() && Lab5.rightMotor.isMoving());
	}

	private static int convertDistance(double radius, double distance1) {
		return (int) ((180.0 * distance1) / (Math.PI * radius));
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
