
import lejos.hardware.Sound;
import lejos.hardware.device.LMotor;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private Port lsPort = LocalEV3.get().getPort("S1");

	private static EV3ColorSensor LSensor;
	private static SampleProvider sensorVal;

	private double actualX;
	private double actualY;

	float[] sensorValData;

	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		this.LSensor = new EV3ColorSensor(lsPort);
		this.sensorVal = LSensor.getRedMode();
		this.sensorValData = new float[LSensor.sampleSize()];
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// read value from light sensor and determine if it is a black line or not
			sensorVal.fetchSample(sensorValData, 0);
			boolean isLine;
			
			int numLinesX = 0;
			int numLinesY = 0;
			
			double offset = 4;
			double offsetY = 5.2;

			double[] ourData = odometer.getXYT();
			
			double initXsensor = -9.2;
			double initYsensor = -14.5;
			
			double initXWB = -13.2;
			double initYWB = -14.5;

			if (sensorValData[0] < 0.20) {
				Sound.beep();
				
				if ((ourData[2] > 0 && ourData[2] <= 5) || ourData[2]>355) {
					odometer.setY(numLinesY*30.48 - offset);
					numLinesY++;
				}
				
				if(ourData[2] <=95 && ourData[2]>85 ) {
					odometer.setX(numLinesX*30.48 - offset);
					numLinesX++;
				}
				
				if (ourData[2] <=185 && ourData[2]>175) {
					odometer.setY(numLinesY*30.48 + offset);
					numLinesY--;
				}
				
				if (ourData[2] <=275 && ourData[2]>265) {
					odometer.setX(numLinesX*30.48 + offset);
					numLinesX--;
				}
				
				if((ourData[2] >= 355) && sensorValData[0] > 0.20 && Lab2.rightMotor.isMoving() == false) {
					double[] result = new double[3];
					result = odometer.getXYT();
					odometer.setXYT(result[0], result[1], result[2]);
					
				}
				
//		    	  if(ourData[2] <= 90 || ourData[2] >= 355){
//		    		  	odometer.setY(YCounter * 30.48);	//correct the Y component
//		    		  	numLinesY++;					//YCounter should increase from 0 to 2
//		    	  		LCD.drawString("YCounter=" + numLinesY, 0, 4);	//use to change the range of Theta 
//		    	  	  	Sound.beep();
//		    	  }
//		      
//		    	  //second side
//		      		else if(odometer.getTheta() > 90 && odometer.getTheta() <= 175){
//		      			odometer.setX(XCounter * 30.48);	//correct the X component
//		      			numLinesX++;
//		      			LCD.drawString("XCounter=" + numLinesX, 0, 4);
//		      			Sound.beep();
//		    	  }
//		      
//		    	  //third side
//		      		else if(odometer.getTheta() > 179 && odometer.getTheta() <= 268){
//		      			odometer.setY((YCounter -1) * 30.48);	//correct the Y component
//		      			numLinesY--;				//YCounter should decrease from 2 to 0
//		      			LCD.drawString("YCounter=" + numLinesY, 0, 4);
//		      			Sound.beep();
//		    	  }
//		      
//		    	  //fourth side
//		      		else if(odometer.getTheta() > 269 && odometer.getTheta() < 356){
//		      			odometer.setX((XCounter -1) * 30.48);	//correct the X component
//		      			numLinesX--;
//		      			LCD.drawString("XCounter=" + numLinesX, 0, 4);
//		      			Sound.beep();
//		    	  }

			}
		
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
