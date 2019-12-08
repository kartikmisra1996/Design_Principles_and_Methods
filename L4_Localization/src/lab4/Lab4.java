package lab4;

import lab4.*;
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

public class Lab4 {

	// Motor Objects, and Robot related parameters
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	//private static final EV3UltrasonicSensor uSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.6;


	private static final Port usPort = LocalEV3.get().getPort("S2");




	public static void main(String[] args) throws OdometerExceptions {

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		int buttonChoice;

		//		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		//		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		//		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		//		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned
		//		UltrasonicPoller usPoller = null;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		//SimpleWF wall = new SimpleWF(); // TODO Complete                                                               // implementation
		Display odometryDisplay = new Display(lcd); // No need to change
		LightLocalizer lLocalizer = new LightLocalizer(odometer);
		UltrasonicPoller usPoller = null;

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Light | US     ", 0, 2);
			lcd.drawString(" Sensor| Sensor ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread lLoc = new Thread(lLocalizer);
			lLoc.start();

		} else {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("Falling| Rising ", 0, 1);
			lcd.drawString(" sensor| sensor ", 0, 2);
			lcd.drawString("       |        ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			if (buttonChoice == Button.ID_LEFT) {
				UltrasonicLocalizer usLocalizer1 = new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocType.FALLING_EDGE);
				usPoller = new UltrasonicPoller (usDistance, usData, usLocalizer1);
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Thread usLoc = new Thread(usLocalizer1);
				usLoc.start();
				usPoller.start();
			}  

			// Start correction if right button was pressed
			else if (buttonChoice == Button.ID_RIGHT) {
				UltrasonicLocalizer usLocalizer2 = new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocType.RISING_EDGE);
				usPoller = new UltrasonicPoller (usDistance, usData, usLocalizer2);
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				Thread usLoc = new Thread(usLocalizer2);
				usLoc.start();
				usPoller.start();
			}
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_DOWN) {
				Thread lLoc2 = new Thread(lLocalizer);
				lLoc2.start();
			}
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}

