package org.usfirst.frc.team6520.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class Pixy {
	
	public int DIGITAL_PORT;
	public int ANALOG_INPUT_PORT;
	public double MAX_VOLTAGE = 3.3;
	public int WIDTH = 320;
	
	public Pixy (int DIO, int AI){
		DIGITAL_PORT = DIO;
		ANALOG_INPUT_PORT = AI;
	}
	
	public DigitalInput DigitalIn = new DigitalInput(DIGITAL_PORT);
	public AnalogInput AnalogIn = new AnalogInput(ANALOG_INPUT_PORT);
	
	public double getObjectX(){
//		if (DigitalIn.get()){
			return AnalogIn.getVoltage()/MAX_VOLTAGE * WIDTH;
//		}
//		return -1;
	}
}
