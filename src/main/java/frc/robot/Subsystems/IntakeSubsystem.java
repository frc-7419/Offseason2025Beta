// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax topMotor;   
  private final SparkMax bottomMotor;

  public IntakeSubsystem() {
    topMotor = new SparkMax(IntakeConstants.motorOneCanID, MotorType.kBrushless);
    bottomMotor = new SparkMax(IntakeConstants.motorTwoCanID, MotorType.kBrushless);

    topMotor.setInverted(false);
    bottomMotor.setInverted(true);
  }

  public void coast() {
    topMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    bottomMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
  }

  public void brake() {
    topMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    bottomMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
  }

  public void setPower(double power) {
    topMotor.set(power);
    bottomMotor.set(power);
  }

  public void runAtVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    bottomMotor.setVoltage(voltage);
  }

  public double getVelocity() {
    return topMotor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Voltage -- Motor One", topMotor.getBusVoltage());
      SmartDashboard.putNumber("Voltage -- Motor Two", bottomMotor.getBusVoltage());
      SmartDashboard.putNumber("Speed -- Motor One", topMotor.get());
      SmartDashboard.putNumber("Speed -- Motor Two", bottomMotor.get());
      SmartDashboard.putNumber("Current Draw -- Motor One", topMotor.getOutputCurrent());
      SmartDashboard.putNumber("Current Draw -- Motor Two", bottomMotor.getOutputCurrent());
  }
}
