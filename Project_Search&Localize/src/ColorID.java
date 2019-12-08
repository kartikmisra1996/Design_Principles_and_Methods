 
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * 
 * The robot will detect the color
 * of a red, blue, white, or yellow block
 * 
 * Authors: Team 03
 * 			Kartik Misra
 * 			Alexis Franche
 * 			Wiam El Ouadi
 * 			Yuliya Volodina
 * 			Saifullah Ahmed
 * 			Jude Habib
 */

public class ColorID implements UltrasonicController, Runnable {
	private int distance;

	Port lsPort = LocalEV3.get().getPort("S4");   
	EV3ColorSensor colorSensor = new EV3ColorSensor(lsPort);
	
	double meanRed[] = new double[3];
	double meanBlue[] = new double[3];
	double meanWhite[] = new double[3];
	double meanYellow[] = new double[3];
	double stdRed[] = new double[3];
	double stdBlue[] = new double[3];
	double stdWhite[] = new double[3];
	double stdYellow[] = new double[3]; 
	private boolean isSearching = true;
	
	
	private static boolean white = false;
	private static boolean yellow = false;
	private static boolean blue = false;
	private static boolean red = false;
	
	public ColorID(Odometer odometer) throws OdometerExceptions{ 
		
	}

	public void run(){
		
		try {
			IDColor();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public void IDColor() throws InterruptedException {
		
		 meanRed[0] = 0.077562778;
		 meanRed[1] = 0.011780444;
		 meanRed[2] = 0.006318778;
		 meanBlue[0] = 0.017428667;
		 meanBlue[1] = 0.025927778;
		 meanBlue[2] = 0.027668444;
		 meanWhite[0] = 0.128322111;
		 meanWhite[1] = 0.148474222;
		 meanWhite[2] = 0.080609667;
		 meanYellow[0] = 0.113724667;
		 meanYellow[1] = 0.099089222;
		 meanYellow[2] = 0.011772333;
		 stdRed[0] = 0.061456085;
		 stdRed[1] = 0.009760975;
		 stdRed[2] = 0.004929478;
		 stdBlue[0] = 0.023989445;
		 stdBlue[1] = 0.019410574;
		 stdBlue[2] = 0.017754152;
		 stdWhite[0] = 0.09324864;
		 stdWhite[1] = 0.115246258;
		 stdWhite[2] = 0.054005774;
		 stdYellow[0] = 0.082772159;
		 stdYellow[1] = 0.079098995;
		 stdYellow[2] = 0.008452954;
		 
		
		SensorMode sensorValID = colorSensor.getRGBMode();
		int sampleSize = sensorValID.sampleSize();  
		float sensorValDataID[] = new float[sampleSize];
		
		while(isSearching) {
			sensorValID.fetchSample(sensorValDataID, 0);
			
			double rVal = sensorValDataID[0];
			double gVal = sensorValDataID[1];
			double bVal = sensorValDataID[2];		
			
			// blue
			if((rVal > meanBlue[0] - stdBlue[0]) && (rVal < meanBlue[0] + stdBlue[0]) && (gVal > meanBlue[1] - stdBlue[1]) && (gVal < meanBlue[1] + stdBlue[1]) && (bVal > meanBlue[2] - stdBlue[2]) && (bVal < meanBlue[2] + stdBlue[2])) {
				LCD.clear();
				LCD.drawString("Object Detected", 0, 4);
				LCD.drawString("   Blue   ", 0, 5);
				blue = true;
				Sound.beep();
				Sound.beep();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					/* 	
					 * there is nothing to be done here because it is not
					 * expected that the ColorID will be
					 * interrupted by another thread 
					 */
				}
			}
			
			// red
			else if((rVal > meanRed[0] - stdRed[0]) && (rVal < meanRed[0] + stdRed[0]) && (gVal > meanRed[1] - stdRed[1]) && (gVal < meanRed[1] + stdRed[1]) && (bVal > meanRed[2] - stdRed[2]) && (bVal < meanRed[2] + stdRed[2])) {
				LCD.clear();
				LCD.drawString("Object Detected", 0, 4);
				LCD.drawString("   Red   ", 0, 5);
				red = true;
				Sound.beep();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					/* 	
					 * there is nothing to be done here because it is not
					 * expected that the ColorID will be
					 * interrupted by another thread 
					 */
				}
			}
			
			// white
			else if((rVal > meanWhite[0] - stdWhite[0]) && (rVal < meanWhite[0] + stdWhite[0]) && (gVal > meanWhite[1] - stdWhite[1]) && (gVal < meanWhite[1] + stdWhite[1]) && (bVal > meanWhite[2] - stdWhite[2]) && (bVal < meanWhite[2] + stdWhite[2])) {
				LCD.clear();
				LCD.drawString("Object Detected", 0, 4);
				LCD.drawString("   White   ", 0, 5);
				white = true;
				Sound.beep();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					/* 	
					 * there is nothing to be done here because it is not
					 * expected that the ColorID will be
					 * interrupted by another thread 
					 */
				}
			}
			
			// yellow
			else if((rVal > meanYellow[0] - stdYellow[0]) && (rVal < meanYellow[0] + stdYellow[0]) && (gVal > meanYellow[1] - stdYellow[1]) && (gVal < meanYellow[1] + stdYellow[1]) && (bVal > meanYellow[2] - stdYellow[2]) && (bVal < meanYellow[2] + stdYellow[2])) {
				LCD.clear();
				LCD.drawString("Object Detected", 0, 4);
				LCD.drawString("   Yellow   ", 0, 5);
				yellow = true;
				Sound.beep();
				

				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					/* 	
					 * there is nothing to be done here because it is not
					 * expected that the ColorID will be
					 * interrupted by another thread 
					 */
				}
			} else {
				LCD.clear();
			}
		}
	}
	
	/**
	 * returns true if target color is detected
	 * @return
	 */
	
	public static boolean redDetected() {
		return red;
	}
	public static boolean whiteDetected() {
		return white;
	}
	public static boolean yellowDetected() {
		return yellow;
	}
	public static boolean blueDetected() {
		return blue;
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
