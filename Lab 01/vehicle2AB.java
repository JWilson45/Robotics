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

public class vehicle2AB {


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


			System.out.println(colorSample1[0] + "   " + (int)(max * colorSample1[0]));


			int leftSpeed = (int)(1.5 * motorLeft.getMaxSpeed() * colorSample4[0]);
			int rightSpeed = (int)(1.5 * motorRight.getMaxSpeed() * colorSample1[0]);

			if (( colorSample4[0] < 0.1 && colorSample1[0] < 0.1 ) || ( colorSample4[0] > 0.7 && colorSample1[0] > 0.7 )) {

				motorLeft.setSpeed(300);
				motorRight.setSpeed(300);
				
			} else {

				if (leftSpeed > 300) {
					leftSpeed = 300;
				}
				if (rightSpeed > 300) {
					rightSpeed = 300;
				}

 				motorLeft.setSpeed(leftSpeed);
				motorRight.setSpeed(rightSpeed);
			}

		}


	}

}
