import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class vehicle2A {

//	final static String SENSOR_PORT_1 = "S1";
//	final static String SENSOR_PORT_4 = "S4";

	static EV3ColorSensor colorSensor;
	static SampleProvider colorProvider;
	static float[] colorSample;

	public static void main(String[] args) {

		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();

//		Port s1 = LocalEV3.get().getPort(SENSOR_PORT_1);
//		colorSensor = new EV3ColorSensor(s1);
//		colorProvider = colorSensor.getRGBMode();
//		colorSample = new float[colorProvider.sampleSize()];

		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);

		while (Button.ESCAPE.isUp()) {
			motorRight.forward();
			motorLeft.forward();
		}

//		colorSensor.close();

	}

}
