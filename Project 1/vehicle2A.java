
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import java.lang.Math;

public class vehicle2A {


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

		// Calibration for the light sensors
		colorProvider1.fetchSample(colorSample1,0);
		float baseSample1 = colorSample1[0];

		colorProvider4.fetchSample(colorSample4,0);
		float baseSample4 = colorSample4[0];

		motorRight.forward();
		motorLeft.forward();

		// main loop - keeps us looking for input
		while (Button.ESCAPE.isUp()) {
			motorRight.forward();
			motorLeft.forward();


			colorProvider1.fetchSample(colorSample1,0);
			colorProvider4.fetchSample(colorSample4,0);


			System.out.println(colorSample1[0] + "   " + (int)(motorRight.getMaxSpeed() * colorSample1[0]));

			int leftSpeed = (int)(motorLeft.getMaxSpeed() * colorSample4[0] * 2);
			int rightSpeed = (int)(motorRight.getMaxSpeed() * colorSample1[0] * 2);

			motorRight.setSpeed(rightSpeed);
			motorLeft.setSpeed(leftSpeed);

			//Delay.msDelay(250);





			//System.out.println(motorRight.getMaxSpeed());


//			while (colorSample1[0] < 0) {
//				motorLeft.setSpeed(100);
//			}
			// while receiving input from sensor1 do this:



			// while receiving input from sensor4 do this:


		}

//		colorSensor.close();

	}

}
