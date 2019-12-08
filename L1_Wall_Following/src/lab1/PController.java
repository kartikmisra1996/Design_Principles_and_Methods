package lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 170;
	private static final int FILTER_OUT = 13;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		if (distance < 13) {
			WallFollowingLab.rightMotor.setSpeed(10);
		}

		else {
			WallFollowingLab.rightMotor.setSpeed(rightWheelFunction(distance));
		}

	}

	//method to increase/decrease speed linearly
	private int rightWheelFunction (int distance) {
		if (distance < 30) {
			return  (int) (8.24*distance-72.4);
		}
		if (distance >30 && distance < 40) {
			return (int) (1.1*distance+120);
		}
		else {
			return 300;
		}
	}



	@Override
	public int readUSDistance() {
		return this.distance;
	}

}