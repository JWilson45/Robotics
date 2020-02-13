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

public class vehicle2A {

	
	// constants for port numbers
	final static String SENSOR_PORT_1 = "S1";
	final static String SENSOR_PORT_4 = "S4";
	
	static EV3ColorSensor colorSensor1;
	static EV3ColorSensor colorSensor4;
	static SampleProvider colorProvider1;
	static SampleProvider colorProvider4;
	static float[] colorSample1;
	static float[] colorSample4;
 	
	@SuppressWarnings("resource")
	public static void main(String[] args) {
		
		EV3 ev3Brick = (EV3) BrickFinder.getLocal();
		Keys buttons = ev3Brick.getKeys();
		
		colorSensor1 = new EV3ColorSensor(LocalEV3.get().getPort(SENSOR_PORT_1));
		colorSensor4 = new EV3ColorSensor(LocalEV3.get().getPort(SENSOR_PORT_4));
		
		colorProvider1 = colorSensor1.getAmbientMode();
		colorProvider4 = colorSensor4.getAmbientMode();
		
		colorSample1 = new float[colorProvider1.sampleSize()];
		colorSample4 = new float[colorProvider4.sampleSize()];
		
		RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
		
		colorProvider1.fetchSample(colorSample1,0);
		float baseSample1 = colorSample1[0];
		
		colorProvider4.fetchSample(colorSample4,0);
		float baseSample4 = colorSample4[0];
		
		// main loop - keeps us looking for input
		while (Button.ESCAPE.isUp()) {
			
			motorRight.forward();
			motorLeft.forward();
			
			colorProvider1.fetchSample(colorSample1,0);
			colorProvider4.fetchSample(colorSample4,0);
			
			// while receiving input from sensor1 do this:

			
			
			// while receiving input from sensor4 do this:
			

		}
		
//		colorSensor.close();
		
	}

}