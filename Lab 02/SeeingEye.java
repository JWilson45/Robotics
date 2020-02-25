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

	final static String SENSOR_PORT_1 = "S1";
	final static String SENSOR_PORT_2 = "S2";

	static NXTUltrasonicSensor wallSensor;
	static NXTUltrasonicSensor forwardSensor;
	static SampleProvider wallDistanceProvider;
	static SampleProvider forwardDistanceProvider;
	static float[] wallDistance;
	static float[] forwardDistance;

	final int topSpeed = 200;
	float kp;
	float kd;
	float ki;

	public static void main(String[] args){

		// Set up for EV3 , Sensors , Motors
		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();


		// Set up for the UltraSonic Sensors
		wallSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_1));
		forwardSensor = new NXTUltrasonicSensor(LocalEV3.get().getPort(SENSOR_PORT_2));

		wallDistanceProvider = wallSensor.getDistanceMode();
		forwardDistanceProvider = forwardSensor.getDistanceMode();

		//  Initialize distance arrays
		wallDistance = new float[wallDistanceProvider.sampleSize()];
		forwardDistance = new float[forwardDistanceProvider.sampleSize()];

		// Set up the Motors to drive the wheels
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);


		while (Button.ESCAPE.isUp()) {

			wallDistanceProvider.fetchSample(wallDistance, 0);

			System.out.println(wallDistance[0]);

		}








	}




	//Correction is proportional to error using direction and magnitude
	//Take in sensor data and compare it to the distance from the wall that we want.
	//Change motor speed to compensate
	//kp (gain) is the multiplier of the speed that the change happens
	public static float returnProportional(float kp) {


	}


	//Produces output that is proportional to the
	//derivative of the input
	public static float returnDiriv(float kd) {


	}





	//System tracks errors, sums them up and once the
	//total error crosses a threshold, the system corrects
	public static float returnSumation(float ki) {


	}

}
