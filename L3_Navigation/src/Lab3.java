
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Motor Objects, and Robot related parameters
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	static final EV3MediumRegulatedMotor servoMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	//private static final EV3UltrasonicSensor uSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.6;


	private static final Port usPort = LocalEV3.get().getPort("S2");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	// this instance
	static float[] usData = new float[usDistance.sampleSize()]; // sample is the buffer in which data are
	// returned



	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		//		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		//		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		//		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		//		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
		//		UltrasonicPoller usPoller = null;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		//SimpleWF wall = new SimpleWF(); // TODO Complete
		Navigate navigate = new Navigate(odometer);                                                                  // implementation
		Display odometryDisplay = new Display(lcd); // No need to change
		Obstacle obstacle = new Obstacle(odometer);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Float | Lab 3  ", 0, 2);
			lcd.drawString("motors |        ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// Float the motors
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			// Display changes in position as wheels are (manually) moved

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

		} else {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString(" Navi- | with   ", 0, 1);
			lcd.drawString(" gate  | obst-  ", 0, 2);
			lcd.drawString("       | acle   ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			if (buttonChoice == Button.ID_LEFT) {
				// Start odometer and display threads
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Thread Navigate = new Thread(navigate);
				Navigate.start();

			}  

			// Start correction if right button was pressed
			else if (buttonChoice == Button.ID_RIGHT) {
				UltrasonicPoller usPoller = new UltrasonicPoller (usDistance, usData, obstacle);
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Thread Obstacle = new Thread(obstacle);
				Obstacle.start();
				usPoller.start();
			}

		}





		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}

