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
import java.util.Date;
import java.util.concurrent.TimeUnit;

public class lab02 {
	
	public static void main(String[] args) {
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		TextLCD lcddisplay = ev3brick.getTextLCD();
		
		Port sidePort = LocalEV3.get().getPort("S2"); 
		Port frontPort = LocalEV3.get().getPort("S1");
		
		RegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		RegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.D);
		float kp = 8000;
		float kd = 20f;
		float ki = 1;
		
		SensorModes sideSensor = new NXTUltrasonicSensor(sidePort);
		SensorModes frontSensor = new NXTUltrasonicSensor(frontPort);
		
		
		SampleProvider sideSampler = sideSensor.getMode("Distance");
		SampleProvider frontSampler = frontSensor.getMode("Distance");
		float[] sideSamples = new float[sideSampler.sampleSize()];
		float[] frontSamples = new float[frontSampler.sampleSize()];
		mLeft.setSpeed(200);
		float timeStart = new Date().getTime();
		float errorTotal = 0;
		while(true) {
			mLeft.backward();
			mRight.backward();
			sideSampler.fetchSample(sideSamples, 0);
			frontSampler.fetchSample(frontSamples, 0);
			float timeEnd = new Date().getTime();
			float timeDiff = (timeEnd - timeStart) *1000;
			float sideReading = sideSamples[0];
			float frontReading = frontSamples[0];
			if(frontReading <= 0.4f) {
				mLeft.setSpeed(0);
				mRight.rotate(420);
				mLeft.setSpeed(200);
			}
			float error = sideReading - 0.3f;
			if(error > 1) {
				error = 1;
			}
			errorTotal += error;
			float integral = errorTotal * ki;
			float derivative = kd * (error / timeDiff);
			int rightSpeed = (int)(200 + kp*error);
			rightSpeed += derivative;
			rightSpeed += integral;
			if(rightSpeed < 150 ) {
				rightSpeed = 150;
			}
			if(rightSpeed > 600) {
				rightSpeed = 600;
			}
			mRight.setSpeed(rightSpeed);
			
			String str = Float.toString(sideReading);
			String str2 = Float.toString(frontSamples[0]);
			lcddisplay.clear();
			lcddisplay.drawInt((int)integral, 2, 4);
			lcddisplay.drawString(str2, 2, 6);
		}
	}
}
