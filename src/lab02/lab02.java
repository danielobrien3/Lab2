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
		float k = 1000;
		
		SensorModes sideSensor = new NXTUltrasonicSensor(sidePort);
		SensorModes frontSensor = new NXTUltrasonicSensor(frontPort);
		
		
		SampleProvider sideSampler = sideSensor.getMode("Distance");
		SampleProvider frontSampler = frontSensor.getMode("Distance");
		float[] sideSamples = new float[sideSampler.sampleSize()];
		float[] frontSamples = new float[frontSampler.sampleSize()];
		mLeft.setSpeed(200);
		while(true) {
			mLeft.backward();
			mRight.backward();
			sideSampler.fetchSample(sideSamples, 0);
			frontSampler.fetchSample(frontSamples, 0);
			float sideReading = sideSamples[0];
			float frontReading = frontSamples[0];
			float error = sideReading - 0.15f;
			int rightSpeed = (int)(200 + k*error);
			if(rightSpeed < 0 ) {
				rightSpeed = 0;
			}
			mRight.setSpeed(rightSpeed);
			/*if(error == 0) {
				mRight.setSpeed(200);
				mLeft.setSpeed(200);
			}
			if(error > 0) {
				float rightSpeed = k*error;
				mRight.setSpeed((int)rightSpeed);
				mLeft.setSpeed(200);
			}
			if(error < 0) {
				float leftSpeed = Math.abs(k*error);
				mLeft.setSpeed((int)leftSpeed);
				mRight.setSpeed(200);
			}*/
			String str = Float.toString(sideReading);
			String str2 = Float.toString(frontSamples[0]);
			lcddisplay.clear();
			lcddisplay.drawString(str, 2, 4);
			lcddisplay.drawInt(rightSpeed, 2, 6);
		}
	}
}
