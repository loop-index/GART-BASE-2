package org.usfirst.frc.team6520.robot.subsystems;

import org.usfirst.frc.team6520.robot.Robot;
import org.usfirst.frc.team6520.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_Drivebase extends Subsystem implements PIDOutput {

    public Spark left = new Spark(1);
    public Spark right = new Spark(0);
//    public Spark pg = new Spark(2);
    
    public Encoder leftEncoder = new CIMCoder(0, 1, false, EncodingType.k4X);
    public Encoder rightEncoder = new CIMCoder(2, 3, true, EncodingType.k4X);
//	public Encoder enc = new Encoder(4,5, true, EncodingType.k4X);
    
    public AHRS navX = new AHRS(SPI.Port.kMXP);
    
    public int PPR = 80;
    public double HIPower = 0.4, LOWPower = 0.4;
    public double rightCoeff = 1.1;
    public boolean arrived;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void driveTimer(double time, int dir){
    	leftEncoder.reset();
    	rightEncoder.reset();
    	double start = Timer.getFPGATimestamp();
    	double acceptableDifference = 5;
    	while (Timer.getFPGATimestamp() - start < time){
    		SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - start);
    		if (leftEncoder.getRaw()/PPR - rightEncoder.getRaw()/PPR > acceptableDifference){
        		left.set(HIPower * dir);
        		right.set(LOWPower * dir);
    		} 
    		else if (rightEncoder.getRaw()/PPR - leftEncoder.getRaw()/PPR > acceptableDifference){
        		left.set(LOWPower * dir);
        		right.set(HIPower * dir);
    		}
    		else {
        		left.set(LOWPower * dir);
        		right.set(LOWPower * dir);
    		}
    		RobotMap.update();
    	}
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public void turn(int angle){
    	navX.reset();
    	leftEncoder.reset();
    	rightEncoder.reset();
    	double turnDiameter = 58;
    	double required = turnDiameter * Math.PI * (angle/360.0);
    	SmartDashboard.putNumber("required dist", required);
    	while (navX.getAngle() < angle){
    		RobotMap.update();
    		left.set(0.25);
    		right.set(0.25);
    	}
    	left.stopMotor();
    	right.stopMotor();
    	
    }
    
    public void turnP(double angle){
    	navX.reset();
    	double heading = navX.getAngle();
    	double speed = 0.6;
    	double kP = 0.08;
    	double E = 0;
    	while (navX.getAngle() - heading < angle){
    		SmartDashboard.putNumber("E", navX.getAngle() - heading);
    		RobotMap.update();
    		E = angle - navX.getAngle();
    		left.set(E/angle * speed * kP);
    		right.set(E/angle * speed * kP);
    	}
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public void followX (int diff){
    	SmartDashboard.putBoolean("arrived?", arrived);
    	if (Robot.vis.avgSize != -1 && Robot.vis.avgSize < 1200){
        	if (Robot.vis.centerX > 160 + diff){
            	left.set(HIPower );
            	right.set(LOWPower * rightCoeff);
        	} else if (Robot.vis.centerX < 160 - diff){
            	left.set(LOWPower );
            	right.set(HIPower  * rightCoeff);
        	} else {
        		left.set(LOWPower );
            	right.set(LOWPower  * rightCoeff);
        	}
    	} else if (Robot.vis.avgSize > 1200){
    		left.stopMotor();
    		right.stopMotor();
    		arrived = true;
    		hitAndRun();
    	}
    	else {
    	}
    }
    
    public void hitAndRun(){
//    	driveTimer(0.5, 1);
    	Timer.delay(2);
    	driveTimer(2, -1);
    	Robot.vis.m_visionThread.interrupt();
    }
    
    
    public void driveByEncoder(double distance){
    	double speed = 0.5;
    	double dist = distance * 100/(15.24 * Math.PI) * PPR;
    	leftEncoder.reset();
    	rightEncoder.reset();
    	while (Math.abs((leftEncoder.get() + rightEncoder.get())/2 - dist) > 10){
    		if ((leftEncoder.get() + rightEncoder.get())/2 < dist){
    			left.set(speed);
        		right.set(speed);
    		} else if ((leftEncoder.get() + rightEncoder.get())/2 > dist){
    			left.set(-speed);
        		right.set(-speed);
    		}
    		SmartDashboard.putNumber("left", leftEncoder.get());
    		SmartDashboard.putNumber("right", rightEncoder.get());
    		SmartDashboard.putNumber("dist", dist);
    	}
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public void driveByEncoderPID(double distance, double Kp, double Ki, double Kd){
    	double speed = 0.5;
    	double dist = distance * 100/(15.24 * Math.PI) * PPR;
    	
    	leftEncoder.reset();
    	rightEncoder.reset();
    	
    	left.setInverted(true);
    	
    	PIDController pidLeft = new PIDController(Kp, Ki, Kd, leftEncoder, left);
    	PIDController pidRight = new PIDController(Kp, Ki, Kd, rightEncoder, right);
    	
    	pidLeft.setOutputRange(-speed, speed);
    	pidRight.setOutputRange(-speed, speed);
    	
    	pidLeft.setAbsoluteTolerance(10);
    	pidRight.setAbsoluteTolerance(10);
    	
    	pidLeft.setSetpoint(dist);
    	pidRight.setSetpoint(dist);
    	
    	pidLeft.enable();
    	pidRight.enable();
    	
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public void turnPID(double angle, double Kp, double Ki, double Kd){
    	navX.reset();
    	PIDController turnController = new PIDController(Kp, Ki, Kd, navX, this);
		turnController.setInputRange(0, 360); // min 0m, max 4m
		turnController.setOutputRange(-0.4, 0.4);
		turnController.setAbsoluteTolerance(2);
		turnController.setContinuous(true);
		turnController.reset();
		turnController.setSetpoint(angle);
		turnController.enable();
		
		while (!turnController.onTarget()){
			left.set(-turnController.get());
			right.set(turnController.get());
			SmartDashboard.putNumber("headinng", navX.getAngle());
		}
		left.stopMotor();
		right.stopMotor();
		SmartDashboard.putNumber("headinng", navX.getAngle());
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}


