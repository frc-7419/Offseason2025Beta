// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class ShooterWrist extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder encoder;
  private double targetPosition = WristConstants.STOW_POSITION;

  public ShooterWrist() {
    wristMotor = new TalonFX(WristConstants.CAN_ID); // Replace 1 with actual CAN ID
    encoder = new DutyCycleEncoder(5);
    configureMotor();
  }

  private void configureMotor() {
    wristMotor.set(0);
  }

  public double getVelocity() {
    return getVelocity();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  /**
   * Sets the power output for the wrist motor.
   *
   * @param power the power level to set, where -1.0 is full reverse,
   *              0.0 is neutral, and 1.0 is full forward.
   */

  public void setPower(double power) {
    wristMotor.set(power);
  }

  public AngularVelocity getVelocityInRadians() {
      return AngularVelocity.ofBaseUnits(getVelocity() * 2 * Math.PI, Units.RadiansPerSecond);
  }


  // public double getCurrentPosition() {
  //   return encoder.getPosition();
  // }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public Angle getPosition() {
    return Angle.ofBaseUnits(encoder.get(), Units.Radians);
  }

  // Brake the motor --> fix this
  public void brake() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coast() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Target", targetPosition);
    SmartDashboard.putNumber("Wrist Rotations", encoder.get());
    SmartDashboard.putNumber("Wrist Velocity", wristMotor.getVelocity().getValueAsDouble());
  }
}
