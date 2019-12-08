

import lejos.hardware.Button;
import lejos.hardware.Sound;

import java.util.concurrent.TimeUnit;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.6;


	private static final Port usPort = LocalEV3.get().getPort("S2");




	@SuppressWarnings({ "deprecation", "static-access" })
	public static void main(String[] args) throws OdometerExceptions {

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		int buttonChoice;


		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		//SimpleWF wall = new SimpleWF(); // TODO Complete                                                               // implementation
		Display odometryDisplay = new Display(lcd); // No need to change
		LightLocalizer lLocalizer = new LightLocalizer(odometer);
		UltrasonicPoller usPoller = null;
		ColorID colIdentifier = new ColorID(odometer);
		Searching search = new Searching(odometer);
		UltrasonicLocalizer usLocalizer1 = new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocType.FALLING_EDGE);
		Thread usLoc = new Thread(usLocalizer1);
		Thread lLoc2 = new Thread(lLocalizer);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("       | US     ", 0, 2);
			lcd.drawString(" Search| Sensor ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			UltrasonicPoller usPoller1 = new UltrasonicPoller (usDistance, usData, search);
			Thread colorIdentifier = new Thread(colIdentifier);
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			Thread searchThread = new Thread (search);
			colorIdentifier.start();
			searchThread.start();
			usPoller1.start(); 
			
			
			


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
				//UltrasonicLocalizer usLocalizer1 = new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocType.FALLING_EDGE);
				usPoller = new UltrasonicPoller (usDistance, usData, usLocalizer1);
				
				Thread odoThread = new Thread(odometer);
				odoThread.start();
				Thread odoDisplayThread = new Thread(odometryDisplay);
				odoDisplayThread.start();
				
				//Thread usLoc = new Thread(usLocalizer1);
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
				//Thread usLoc = new Thread(usLocalizer2);
				usLoc.start();
				usPoller.start();
			}
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_DOWN) {
				usPoller.interrupt();
				usLoc.interrupt();
				
				lLoc2.start();
				
				
				
				
			}
			
			buttonChoice = Button.waitForAnyPress();
//			try {
//				lLoc2.sleep(999999);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
			if(buttonChoice == Button.ID_UP) {
			
			
				Sound.twoBeeps();
				
			UltrasonicPoller usPoller1 = new UltrasonicPoller (usDistance, usData, search);
			Thread searchThread = new Thread (search);
			lLocalizer.isDone = true;
			Thread colorIdentifier = new Thread(colIdentifier);
			colorIdentifier.start();
			searchThread.start();
			usPoller1.start();
			}
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}

