package org.usfirst.frc.team6520.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;

public class CIMCoder extends Encoder{
	
	public int encodingMultiplier = 4;

	public CIMCoder(int i, int j, boolean reverseDirection, EncodingType encodingType) {
		super(i, j, reverseDirection, encodingType);
	}
	
	public int PPR = 20;
	public double diameter = 15.48;

	public double getDistance(){
		return super.getRaw() * diameter * Math.PI * (1.0/PPR) / encodingMultiplier;
	}
}
