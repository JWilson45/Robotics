import lejos.hardware.ev3.EV3;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.lcd.TextLCD;


public class HelloWorld {
	
	public static void main(String[] args) {
		// get EV3 Brick
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
		TextLCD lcddisplay = ev3brick.getTextLCD();
		
		lcddisplay.drawString("HelloWorld", 2, 4);
		
		buttons.waitForAnyPress();

	}

}
