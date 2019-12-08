
/*
 * The robot will navigate to each coordinate without avoiding obstacles
 * 
 * Author: Alexis Franche, Kartik Misra
 */

public class Navigate extends Thread {
	double wheel_radius = Lab3.WHEEL_RAD;
	double width = Lab3.TRACK;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private final double TILE_SIZE = 30.48;
	public double odo_x,odo_y, odo_theta;
	
	public Odometer odometer;
	public Navigate(Odometer odometer){ //constructor
		this.odometer = odometer;
	}
	
	public void run(){
		travelTo(1,1);
		travelTo(0,2);
		travelTo(2,2);
		travelTo(2,1);
		travelTo(1,0);
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
		
		double finalX = pointX * TILE_SIZE;
		double finalY = pointY * TILE_SIZE;
		

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
		Lab3.leftMotor.setSpeed(FORWARD_SPEED);
		Lab3.rightMotor.setSpeed(FORWARD_SPEED);
						
		Lab3.leftMotor.rotate(convertDistance(wheel_radius, distance), true);
		Lab3.rightMotor.rotate(convertDistance(wheel_radius, distance), false);
	}
	
	
	/*Authors: Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to rotate to the input distance
	 * 
	 * @param angle to turn to
	 * @void command the robot wheels to move to reach input distance
	 */
	public void turnTo(double theta){
		Lab3.leftMotor.setSpeed(ROTATE_SPEED);
		Lab3.rightMotor.setSpeed(ROTATE_SPEED);
		
		Lab3.leftMotor.rotate(convertAngle(wheel_radius, width, theta), true);
		Lab3.rightMotor.rotate(-convertAngle(wheel_radius, width, theta), false);
	}
	
	
	/*Author: Alexis Franche
	 * 
	 * Checks if the wheels are moving, if yes return true, false otherwise
	 * 
	 * @return 
	 * 
	 */
	public boolean isNavigating(){
		return (Lab3.leftMotor.isMoving() && Lab3.rightMotor.isMoving());
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}