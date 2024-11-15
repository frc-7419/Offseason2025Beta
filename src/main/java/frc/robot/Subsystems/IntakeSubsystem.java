// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  Spark motorOne;
  Spark motorTwo;

  public IntakeSubsystem(Spark motorOne, Spark motorTwo) {
    motorOne = new Spark(IntakeConstants.motorOneCanID);
    motorTwo = new Spark(IntakeConstants.motorTwoCanID);
    // NO need for add requirments    
  }
// Figure out the coast mode
  public void coast(){
    
  }

  public void brake(){

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
