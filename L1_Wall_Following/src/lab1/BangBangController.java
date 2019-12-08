package lab1;


import lejos.hardware.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(150); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(150);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;

		float error = bandCenter - distance;
		//range to go straight
		if ((distance < bandCenter+bandwidth) && (distance >= bandCenter-bandwidth)) {
			WallFollowingLab.leftMotor.setSpeed(200);
			WallFollowingLab.rightMotor.setSpeed(200);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//fairly far
		else if (error < 3 && distance > 70) {
			WallFollowingLab.leftMotor.setSpeed(120);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//very far
		else if (error < 3) {
			WallFollowingLab.leftMotor.setSpeed(100);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		} 
		//fairly close
		else if (error > 3 && distance < 13) {
			WallFollowingLab.leftMotor.setSpeed(400);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.backward();
			WallFollowingLab.leftMotor.forward();
		}
		//very close
		else if (error > -3) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//small/convex angles
		else if (distance > 255) {
			WallFollowingLab.leftMotor.setSpeed(300);
			WallFollowingLab.rightMotor.setSpeed(300);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();
		}
		//go backward for imminent collision
		else if (distance < 13) {
			WallFollowingLab.leftMotor.setSpeed(150);
			WallFollowingLab.rightMotor.setSpeed(150);
			WallFollowingLab.leftMotor.backward();
			WallFollowingLab.rightMotor.backward();
		}
	}
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}