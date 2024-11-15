// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
public class ShooterWrist extends SubsystemBase {
  private final SparkMax wristMotor;
  private final RelativeEncoder encoder;
  private double targetPosition = WristConstants.STOW_POSITION;

  public ShooterWrist() {
    wristMotor = new SparkMax(WristConstants.CAN_ID, MotorType.kBrushless); // Replace 1 with actual CAN ID
    encoder = wristMotor.getEncoder();
    configureMotor();
  }

  private void configureMotor() {
    wristMotor.set(0);
    encoder.setPosition(0);
  }

  public double getVelocity() {
    return getVelocity();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  public void setPower(double power) {
    wristMotor.set(power);
  }

  public double getVelocityInRadians() {
      return getVelocity() * 2 * Math.PI;
  }

  public double getCurrentPosition() {
    return encoder.getPosition();
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public boolean isAtPosition() {
    return Math.abs(getCurrentPosition() - targetPosition) < 0.02; // Within 2% of target
  }

  public void brake() {
    wristMotor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake), null, null);
  }

  public void coast() {
    wristMotor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast), null, null);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getCurrentPosition());
    SmartDashboard.putNumber("Wrist Target", targetPosition);
    SmartDashboard.putNumber("Wrist Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Wrist Current", wristMotor.getOutputCurrent());
  }
}
