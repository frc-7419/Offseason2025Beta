// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  PWMSparkMax motorOne;   // Should use Spark?
  PWMSparkMax motorTwo;


  public IntakeSubsystem(Spark motorOne, Spark motorTwo) {
    motorOne = new PWMSparkMax(IntakeConstants.motorOneCanID);
    motorTwo = new PWMSparkMax(IntakeConstants.motorTwoCanID);
    motorOne.setInverted(False);
    motorTwo.setInverted(True);
    // NO need for add requirments    
  }
// Figure out the coast mode
  public void coast(){
    
  }

  public void brake(){
    motorOne.stopMotor();
    motorTwo.stopMotor();
  }

  public void runAtVoltage(double voltage){
    motorOne.setVoltage(voltage);
    motorTwo.setVoltage(voltage);
  }

  public void runAtVelocity(double velocity){
    //Use PID
  }


  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Motor One Intake - Temperature (In Farenheit)", )
  //    SmartDashboard.putNumber("Current Draw -- Motor One", motorOne.getOutputCurrent());
      SmartDashboard.putNumber("Voltage -- Motor One", motorOne.getVoltage());
      SmartDashboard.putNumber("Voltage -- Motor Two", motorTwo.getVoltage());
      SmartDashboard.putNumber("Speed -- Motor One", motorOne.get());
      SmartDashboard.putNumber("Speed -- Motor wo", motorTwo.get());
  }
}
