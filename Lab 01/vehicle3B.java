import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class vehicle3B {


	// constants for port numbers
	final static String SENSOR_PORT_1 = "S1";
	final static String SENSOR_PORT_4 = "S4";

	// Variables for EV3 and Color Sensors
	static EV3ColorSensor colorSensor1;
	static EV3ColorSensor colorSensor4;
	static SampleProvider colorProvider1;
	static SampleProvider colorProvider4;
	static float[] colorSample1;
	static float[] colorSample4;


	@SuppressWarnings("resource")
	public static void main(String[] args) {

		// Set up for EV3 , Sensors , Motors
		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();

		// Set up for light sensors
		colorSensor1 = new EV3ColorSensor(LocalEV3.get().getPort(SENSOR_PORT_1));
		colorSensor4 = new EV3ColorSensor(LocalEV3.get().getPort(SENSOR_PORT_4));

		// Set the color sensors to AmbientMode for light intensity measurements
		colorProvider1 = colorSensor1.getAmbientMode();
		colorProvider4 = colorSensor4.getAmbientMode();

		//  Set the color sample array
		colorSample1 = new float[colorProvider1.sampleSize()];
		colorSample4 = new float[colorProvider4.sampleSize()];

		// Set up the Motors to drive the wheels
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);


		// main loop - keeps us looking for input
		while (Button.ESCAPE.isUp()) {

			motorRight.forward();
			motorLeft.forward();

			colorProvider1.fetchSample(colorSample1,0);
			colorProvider4.fetchSample(colorSample4,0);

			System.out.println(colorSample1[0] + "   " + (int)(colorSample1[0]));
			System.out.println(colorSample4[0] + "   " + (int)(colorSample4[0]));

			// Calculates the proportion for the speed of the motors based on light input
			int leftSpeed = (int)(motorLeft.getMaxSpeed() * colorSample4[0]);
			int rightSpeed = (int)(motorRight.getMaxSpeed() * colorSample1[0]);

			// Set the speed of the motors to the calculated proportions
			// set to variables leftSpeed, rightSpeed
			// This will pull the robot to the light source
			motorLeft.setSpeed(leftSpeed + 50);
			motorRight.setSpeed(rightSpeed + 50);

			// Once the robot gets close to the light,
			// turn away and look for a new light source
			if(colorSample4[0] > 0.5 || colorSample1[0] > 0.5 ) {
				robot3b(motorLeft, motorRight);

				// Speed limiter for rob
				if (leftSpeed > 300) {
					leftSpeed = 300;
				}
				if (rightSpeed > 300) {
					rightSpeed = 300;
				}

			}


	}

}


	private static void robot3b(RegulatedMotor left, RegulatedMotor right) {

		left.setSpeed(250);
		right.setSpeed(100);
		left.forward();
		right.forward();
		Delay.msDelay(1000);


	}

}
