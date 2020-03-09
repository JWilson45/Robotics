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

		double kp = 12.0;
		double kd = 2.5;
		double ki = 0.8;


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
			//System.out.println("Iterm = " + ki + "//" + error);//leftSensorData);
			//System.out.println("TotalE: " + totalError);
			motorSpeed = returnMotorSpeed(kp, ki, kd);

			//System.out.println("SDP: " + motorSpeed);

//			motorLeft.setSpeed(topSpeed);
//			motorRight.setSpeed(topSpeed);

			if(motorSpeed > .2) {
				motorLeft.setSpeed(Math.abs(motorSpeed) + topSpeed);
			} else if (motorSpeed < - .2){
				motorRight.setSpeed(Math.abs(motorSpeed) + topSpeed);
			} else {
				motorLeft.setSpeed(topSpeed);
				motorRight.setSpeed(topSpeed);
			}

			//if((frontSensorData * 100 < 40))
			//		motorLeft.setSpeed(Math.abs(motorSpeed) + topSpeed);

			motorRight.forward();
			motorLeft.forward();

			//System.out.println("LSD: " + leftSensorData + "MSPD: " + motorSpeed);
		}
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
	 *
	 * @param sensorData
	 */
	public static void calculateError(Float sensorData) {

		// Error calculation: excpectedValue - actualValue
		lastError = error;
		error = distanceFromWall - (sensorData * 100);

		System.out.println((int)error);

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
	 *
	 * @param kp -- value to be multiplied to the proportional term of the PID loop
	 * @param ki -- value to be multiplied to the Integral term of the PID loop
	 * @param kd -- value to be multiplied to the Derivative calculation of the PID loop
	 * @return
	 */
	public static int returnMotorSpeed(double kp, double ki, double kd) {
		double proportional = error * kp;
		double integral = totalError * ki;
		double derivative = (error - lastError ) * kd;

		double pidOutput = proportional + integral + derivative;

		return (int)pidOutput;

	}

//	/**
//	 * spinMotor
//	 *
//	 * @param motorToSpin -- the string name of the left or right motor we want to manipulate
//	 * @param motor -- the created motor object we want to manipulate
//	 * @param motorSpeed -- the motor speed after it has been altered by the PID controller
//	 */
//	public static void spinMotor(String motorToSpin, RegulatedMotor motor, int motorSpeed) {
//		if(motorToSpin == "RIGHT") {
//			motor.setSpeed(Math.abs(motorSpeed));
//			motor.forward();
//		}
//
//		if(motorToSpin == "LEFT") {
//			motor.setSpeed(Math.abs(motorSpeed));
//			motor.forward();
//		}
//
//	}

//	/**
//	 * returnProportional
//	 *
//	 * @param kp -- (gain) is the multiplier of the speed that the change happens
//	 * @param sensorData -- the sensorData used to calculate the error (actualValue)
//	 * @return -- the proportional value used to assist in setting motor speed
//	 *
//	 * correction is proportional to error using direction and magnitude
//	 */

//	public static double returnProportional(double kp) {
//		return error * kp;
//	}
//	/**
//	 *
//	 * @param ki
//	 * @param sensorData
//	 * @return
//	 */
//	public static double returnDerivative(double ki) {
//		return lastError - error * ki;
//	}

}
