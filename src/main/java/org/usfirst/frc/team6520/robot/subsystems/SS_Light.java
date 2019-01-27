/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6520.robot.subsystems;

import org.usfirst.frc.team6520.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SS_Light extends Subsystem {
  DigitalInput out1 = new DigitalInput(9);
  DigitalInput out2 = new DigitalInput(8);
  DigitalInput out4 = new DigitalInput(7);
  DigitalInput out5 = new DigitalInput(6);

  double HIPower = 0.5;
  double LOWPower = 0.3;

  public String readInput(){
    String line = "";
    line = "" + cvt(out1) + cvt(out2) + cvt(out4) + cvt(out5);
    SmartDashboard.putString("line", line);
    return line;
  }

  public int cvt(DigitalInput dig){
    return (dig.get()) ? 1 : 0;
  }

  public void trackLine(){
    if (readInput().charAt(0) == '0' && readInput().charAt(3) == '0'){
      RobotMap.drivebase.left.set(LOWPower);
      RobotMap.drivebase.right.set(LOWPower);
    } else if (readInput().charAt(0) == '1' && readInput().charAt(3) == '0'){
      RobotMap.drivebase.left.set(HIPower);
      RobotMap.drivebase.right.set(LOWPower);
    } else if (readInput().charAt(0) == '0' && readInput().charAt(3) == '1'){
      RobotMap.drivebase.left.set(LOWPower);
      RobotMap.drivebase.right.set(HIPower);
    } 
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
