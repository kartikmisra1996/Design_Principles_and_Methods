package lab4;

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
import lab4.*;

/*
 * The robot will locate the (0,0) and 0 degree point precisely
 * by scanning grid lines 
 * 
 * Authors: Kartik MIsra, Alexis Franche
 */

public class LightLocalizer implements UltrasonicController, Runnable {
	private int distance;
	private double angleA, angleB;


	double wheel_radius = Lab4.WHEEL_RAD;
	double width = Lab4.TRACK;
	private static final int FORWARD_SPEED = 50;
	private static final int ROTATE_SPEED = 50;
	private static final double OFFSET = 14.2;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;

	private double point1, point2, point3, point4;
	private double thetaY, thetaX, thetaFinal;
	private double xPos, yPos;

	private static Lock lock = new ReentrantLock(true);
	private volatile boolean isReseting = false;
	private Condition doneReseting = lock.newCondition();

	public Odometer odometer;

	private Port lsPort = LocalEV3.get().getPort("S3");

	private static EV3ColorSensor LSensor;
	private static SampleProvider sensorVal;

	float[] sensorValData;

	public LightLocalizer(Odometer odometer) throws OdometerExceptions{ //constructor
		this.odometer = Odometer.getOdometer();
		this.LSensor = new EV3ColorSensor(lsPort);
		this.sensorVal = LSensor.getRedMode();
		this.sensorValData = new float[LSensor.sampleSize()];
	}

	public void run(){
		try {
			lightLocalizer();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/*
	 * Authors : Kartik Misra, Alexis Franche
	 * 
	 * Localization using the light sensor, after completing a 360 degree rotation
	 * and scanning the grid lines the robot will find the origin of the board
	 * 
	 * @void
	 */
	public void lightLocalizer() throws InterruptedException {
		Lab4.leftMotor.stop();
		Lab4.rightMotor.stop();
		odometer.setXYT(0, 0, 0);
		TimeUnit.SECONDS.sleep(2);
		double turnedAngle, finalAngle;
		int lineCount = 0;
		while(lineCount < 4) {
			while (true) {
				sensorVal.fetchSample(sensorValData, 0);
				Lab4.leftMotor.forward();
				Lab4.rightMotor.backward();
				Lab4.rightMotor.setSpeed(50);
				Lab4.leftMotor.setSpeed(50);

				if (sensorValData[0] < 0.27) {
					Sound.beep();
					lineCount++;
					if(lineCount == 1) point1 = odometer.getTheta();
					if(lineCount == 2) point2 = odometer.getTheta();
					if(lineCount == 3) point3 = odometer.getTheta();
					if(lineCount == 4) point4 = odometer.getTheta();
				}

				if(odometer.getTheta() > 358 && odometer.getTheta() <= 360) {
					Lab4.leftMotor.stop();
					Lab4.rightMotor.stop();
					break;
				}
			}
		}
		turnedAngle = odometer.getTheta();

		thetaX = point4 - point2;
		thetaY = point3 - point1;

		xPos = -OFFSET*(Math.cos(Math.PI*thetaX/360));
		yPos = -OFFSET*(Math.cos(Math.PI*thetaY/360));

		thetaFinal = 180 - thetaY/2 - point1;

		odometer.setXYT(xPos, yPos, 0);

		travelTo(0,0);
		turnTo(305);
	}

	public void turnTo(double theta){
		Lab4.leftMotor.setSpeed(50);
		Lab4.rightMotor.setSpeed(50);

		Lab4.leftMotor.rotate(convertAngle(wheel_radius, width, theta), true);
		Lab4.rightMotor.rotate(-convertAngle(wheel_radius, width, theta), false);
	}

	/*
	 * Author: Kartik Mira, ALexis Franche
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction 
	 * 
	 * @param x and y values to travel to
	 * @void commands the robot to rotate and set the travel distance to reach destination
	 * 
	 */
	public void travelTo(double pointX, double pointY){


		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];


		odo_x = actualX;
		odo_y = actualY;
		odo_theta = acutalTheta;

		double finalX = pointX;
		double finalY = pointY;


		double yDiff = finalY-odo_y;
		double xDiff = finalX-odo_x;

		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));

		double distFinal = Math.hypot(xDiff,yDiff);

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
		goTo(distFinal);
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
		Lab4.leftMotor.setSpeed(FORWARD_SPEED);
		Lab4.rightMotor.setSpeed(FORWARD_SPEED);

		Lab4.leftMotor.rotate(convertDistance(wheel_radius, distance), true);
		Lab4.rightMotor.rotate(convertDistance(wheel_radius, distance), false);
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
