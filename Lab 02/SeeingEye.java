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

	static NXTUltrasonicSensor wallSensor;
	static NXTUltrasonicSensor forwardSensor;
	static SampleProvider wallDistanceProvider;
	static SampleProvider forwardDistanceProvider;
	static float[] wallDistance;
	static float[] forwardDistance;

	final static int topSpeed = 200;
	final static int distanceFromWall = 30;


	public static void main(String[] args) {

		double kp = 1.9
				;
		double kd;
		double ki;

		// Set up for EV3 , Sensors , Motors
		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();


		// Set up for the UltraSonic Sensors
		wallSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_4));
		forwardSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_2));


		wallDistanceProvider = wallSensor.getDistanceMode();
		forwardDistanceProvider = forwardSensor.getDistanceMode();

		//  Initialize distance arrays
		wallDistance = new float[wallDistanceProvider.sampleSize()];
		forwardDistance = new float[forwardDistanceProvider.sampleSize()];

		// Set up the Motors to drive the wheels
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);

		motorRight.setSpeed(topSpeed);
		motorLeft.setSpeed(topSpeed);

		while (Button.ESCAPE.isUp()) {

			motorRight.forward();
			motorLeft.forward();

			wallDistanceProvider.fetchSample(wallDistance, 0);

			double value = returnProportional(kp);

			System.out.println(wallDistance[0] * 100 + " " + value);

//			double prop = 300 - Math.abs(value);
//			if(value <= 0){
//
//				motorLeft.setSpeed((int)(topSpeed + prop));
//				motorRight.setSpeed((int)(topSpeed + value));
//			}
//			else {
//				motorRight.setSpeed((int)(topSpeed + prop));
//				motorLeft.setSpeed((int)(topSpeed + value));
//
//			}



		}
	}

	public static void wheelGo(RegulatedMotor motorRight, RegulatedMotor motorLeft, double value) {

	}

	//Correction is proportional to error using direction and magnitude
	//Take in sensor data and compare it to the distance from the wall that we want.
	//Change motor speed to compensate
	//kp (gain) is the multiplier of the speed that the change happens
	public static double returnProportional(double kp) {

		// Error calculation: excpecedValue - actualValue
		double error = distanceFromWall - (wallDistance[0] * 100);

		return error * kp;


	}


	//Produces output that is proportional to the
	//derivative of the input
//	public static double returnDiriv(double kd) {
//
//
//	}

//	public static int forwardSensorValue() {
//
//
//	}



	//System tracks errors, sums them up and once the
	//total error crosses a threshold, the system corrects
//	public static double returnSumation(double ki) {
//
//
//	}

}
