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
import lejos.utility.Delay;


public class SeeingEye {

	final static String SENSOR_PORT_4 = "S4";
	final static String SENSOR_PORT_2 = "S2";
	final static String LEFT_MOTOR    = "LEFT";
	final static String RIGHT_MOTOR   = "RIGHT";

	final static int topSpeed = 250;
	final static int distanceFromWall = 30;
	static double lastError = 0;
	static double error = 0;
	static double totalError = 0;


	public static void main(String[] args) {
		int motorSpeed;

		double kp = 9.0;//30
		double kd = 4.6;//3.6
		double ki = 1;// .4


		// Set up for the UltraSonic Sensors
		NXTUltrasonicSensor wallSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_4));
		NXTUltrasonicSensor forwardSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_2));


		// set US sensors to distance mode
		SampleProvider wallDistanceProvider = wallSensor.getDistanceMode();
		SampleProvider forwardDistanceProvider = forwardSensor.getDistanceMode();


		// Set up the Motors to drive the wheels
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.B);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		//motorLeft.setSpeed(topSpeed);
		//motorRight.setSpeed(topSpeed);

		while (Button.ESCAPE.isUp()) {

			// Tuning code
			if (Button.ENTER.isDown()) {
				motorLeft.stop();
				motorRight.stop();

				Delay.msDelay(1000);

				while (Button.ENTER.isUp()) {
					if (Button.DOWN.isDown()) {
						ki -= 1;
						Delay.msDelay(200);
						System.out.println("Pterm = " + ki);
					}
					if (Button.LEFT.isDown()) {
						ki -= .1;
						Delay.msDelay(200);
						System.out.println("Pterm = " + ki);
					}
					if (Button.RIGHT.isDown()) {
						ki += .1;
						Delay.msDelay(200);
						System.out.println("Pterm = " + ki);
					}
					if (Button.UP.isDown()) {
						ki += 1;
						Delay.msDelay(200);
						System.out.println("Pterm = " + ki);
					}

				}
				Delay.msDelay(1000);
			}
			//end tuning code

			// creates variables to be used while making calculations in the PIDController
			float leftSensorData = pollLeftSensor(wallDistanceProvider);
			float frontSensorData = pollFrontSensor(forwardDistanceProvider);
			calculateError(leftSensorData);

			// Get the motor speed from the pid loop
			motorSpeed = returnMotorSpeed(kp, ki, kd);


			// Set the motor speed
			if(motorSpeed > .2) {
				motorLeft.setSpeed(Math.abs(motorSpeed) + topSpeed);
			} else if (motorSpeed < - .2){
				motorRight.setSpeed(Math.abs(motorSpeed) + topSpeed);
			} else {
				motorLeft.setSpeed(topSpeed);
				motorRight.setSpeed(topSpeed);
			}


			// Reverse code that broke our project for the demo due to the broken forward facing sensor
//			if(frontSensorData * 100 < 21)
//			{
//				motorLeft.setSpeed(-200);
//				motorRight.setSpeed(-200);
//				motorRight.backward();
//				motorLeft.backward();
//				Delay.msDelay(3000);
//				motorLeft.setSpeed(300);
//				motorRight.setSpeed(100);
//				motorRight.forward();
//				motorLeft.forward();
//				Delay.msDelay(3500);
//
//			}

			// apply changes to motors
			System.out.println(leftSensorData *100);
			motorRight.forward();
			motorLeft.forward();
		}

		// Close sensors
		wallSensor.close();
		forwardSensor.close();
		motorRight.close();
		motorLeft.close();

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
	 * Calculates the error and total error for PID calculation
	 *
	 * @param sensorData
	 */
	public static void calculateError(Float sensorData) {

		// Error calculation: excpectedValue - actualValue
		lastError = error;
		error = distanceFromWall - (sensorData * 100);

		//System.out.println((int)error);

		if(totalError > 0 && error < 0)
			totalError = 0;

		if(totalError < 0 && error > 0)
			totalError = 0;

		if(error < 60 || error > -60)
			totalError += error;

		if(totalError == 100)
			totalError = 100;
		if(totalError == -100)
			totalError = -100;
	}

	/**
	 * PID Loop method
	 *
	 * @param kp -- value to be multiplied to the proportional term of the PID loop
	 * @param ki -- value to be multiplied to the Integral term of the PID loop
	 * @param kd -- value to be multiplied to the Derivative calculation of the PID loop
	 * @return output of the pidloop
	 */
	public static int returnMotorSpeed(double kp, double ki, double kd) {
		double proportional = error * kp;				// P
		double integral = totalError * ki;				// I
		double derivative = (error - lastError ) * kd;	// D

		double pidOutput = proportional + integral + derivative; // add the calculations together

		return (int)pidOutput;

	}

}
