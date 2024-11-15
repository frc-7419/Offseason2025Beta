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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax motorOne;   
  private final SparkMax motorTwo;



  public IntakeSubsystem() {
    motorOne = new SparkMax(IntakeConstants.motorOneCanID, IntakeConstants.motorOneType);
    motorTwo = new SparkMax(IntakeConstants.motorTwoCanID, IntakeConstants.motorTwoType);
  }
    //Inversion ?
/*     motorOne.setInverted(False);
    motorTwo.setInverted(True);  */

// Figure out the coast mode
  public void coast(){
    motorOne.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    motorTwo.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
  }

  public void brake(){
    motorOne.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    motorTwo.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
  }
  public void setPower(double power){
    motorOne.set(power);
    motorTwo.set(power);
  }

  public void runAtVoltage(double voltage){
    motorOne.setVoltage(voltage);
    motorTwo.setVoltage(voltage);
  }

  public double getVelocity(){
    // Get the velocity
    
  }


  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Motor One Intake - Temperature (In Farenheit)", )
      SmartDashboard.putNumber("Voltage -- Motor One", motorOne.getBusVoltage());
      SmartDashboard.putNumber("Voltage -- Motor Two", motorTwo.getBusVoltage());
      SmartDashboard.putNumber("Speed -- Motor One", motorOne.get());
      SmartDashboard.putNumber("Speed -- Motor wo", motorTwo.get());
      SmartDashboard.putNumber("Current Draw -- Motor One", motorOne.getOutputCurrent());
      SmartDashboard.putNumber("Current Draw -- Motor Two", motorTwo.getOutputCurrent());
  }
}
