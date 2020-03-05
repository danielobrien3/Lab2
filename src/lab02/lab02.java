package lab02;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.I2CSensor;

public class lab02 {
	
	public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		TextLCD lcddisplay = ev3brick.getTextLCD();
		
		Port sidePort = LocalEV3.get().getPort("S2"); 
		Port frontPort = LocalEV3.get().getPort("S1");
		
		RegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.D);
	
		
		SensorModes sideSensor = new NXTUltrasonicSensor(sidePort);
		SensorModes frontSensor = new NXTUltrasonicSensor(frontPort);
		
		SampleProvider sideSampler = sideSensor.getMode("Distance");
		float[] sideSamples = new float[sideSampler.sampleSize()];
		while(true) {
			String str = Float.toString(sideSamples[0]);
			lcddisplay.drawString(str, 2, 4);
		}
	}
}
