import lejos.hardware.ev3.EV3;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;


public class SeeingEye {

	final static String SENSOR_PORT_4 = "S4";
	final static String SENSOR_PORT_2 = "S2";
	final static String LEFT_MOTOR    = "LEFT";
	final static String RIGHT_MOTOR   = "RIGHT";
	
	final static int topSpeed = 200;
	final static int distanceFromWall = 30;


	public static void main(String[] args) {
		float motorSpeed;
		
		double kp = 1.9;
		double kd;
		double ki;


		// Set up for the UltraSonic Sensors
		NXTUltrasonicSensor wallSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_4));
		NXTUltrasonicSensor forwardSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_2));

		
		// set US sensors to distance mode
		SampleProvider wallDistanceProvider = wallSensor.getDistanceMode();
		SampleProvider forwardDistanceProvider = forwardSensor.getDistanceMode();

		
		// Set up the Motors to drive the wheels
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);

		while (Button.ESCAPE.isUp()) {
			
			spinMotor(RIGHT_MOTOR, motorRight, topSpeed);
			spinMotor(LEFT_MOTOR, motorLeft, topSpeed);
			
			// creates variables to be used while making calculations in the PIDController
			float leftSensorData = pollLeftSensor(wallDistanceProvider);
			float frontSensorData = pollFrontSensor(forwardDistanceProvider);
			
			motorSpeed = (float) returnProportional(kp, leftSensorData);
			
			if(motorSpeed > 0) {
				spinMotor(RIGHT_MOTOR, motorRight, (int) motorSpeed);

			} 
			
			if (motorSpeed < 0 ){
				spinMotor(LEFT_MOTOR, motorLeft, (int) motorSpeed);
			}
			
			System.out.println("LSD: " + leftSensorData + "MSPD: " + motorSpeed);			

		}
		
		wallSensor.close();
		forwardSensor.close();
		
	}
	
	
	/**
	 * spinMotor
	 * 
	 * @param motorToSpin -- the string name of the left or right motor we want to manipulate
	 * @param motor -- the created motor object we want to manipulate
	 * @param motorSpeed -- the motor speed after it has been altered by the PID controller
	 */
	public static void spinMotor(String motorToSpin, RegulatedMotor motor, int motorSpeed) {
		if(motorToSpin == "RIGHT") {
			motor.setSpeed(Math.abs(motorSpeed));
			motor.forward();
		}
		
		if(motorToSpin == "LEFT") {
			motor.setSpeed(Math.abs(motorSpeed));
			motor.forward();
		}
		
	}
	
	
	/**
	 * pollLeftSensor
	 * 
	 * @param wallDistanceProvider -- the left wallDistanceProvider created outside of the main loop
	 * @return -- the value(distance) found by the left facing sensor
	 * 
	 * we do not create the wallDistanceProvider within this function to avoid creating one every  
	 * time the main loop calls this function
	 */
	public static float pollLeftSensor(SampleProvider wallDistanceProvider) {
		float[] wallDistance;
		wallDistance = new float[wallDistanceProvider.sampleSize()];
		wallDistanceProvider.fetchSample(wallDistance, 0);		
		
		return wallDistance[0];
	}
	
	
	/**
	 * pollFrontSensor
	 * 
	 * @param wallDistanceProvider -- the front wallDistanceProvider created outside of the main loop
	 * @return -- the value(distance) found by the front facing sensor
	 * 
	 * we do not create the wallDistanceProvider within this function to avoid creating one every  
	 * time the main loop calls this function
	 */
	public static float pollFrontSensor(SampleProvider forwardDistanceProvider) {
		float[] forwardDistance;
		forwardDistance = new float[forwardDistanceProvider.sampleSize()];
		forwardDistanceProvider.fetchSample(forwardDistance, 0);
		
		return forwardDistance[0];
	}
	

	/**
	 * returnProportional
	 * 
	 * @param kp -- (gain) is the multiplier of the speed that the change happens
	 * @param sensorData -- the sensorData used to calculate the error (actualValue)
	 * @return -- the proportional value used to assist in setting motor speed
	 * 
	 * correction is proportional to error using direction and magnitude
	 */
	public static double returnProportional(double kp, Float sensorData) {

		// Error calculation: excpectedValue - actualValue
		double error = distanceFromWall - (sensorData * 100);

		return error * kp;
	}

}
