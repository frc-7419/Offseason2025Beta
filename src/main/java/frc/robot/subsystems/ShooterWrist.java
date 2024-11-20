// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public void setPower(double power) {
    wristMotor.set(power);
  }

  public double getVelocityInRadians() {
      return getVelocity() * 2 * Math.PI;
  }


  // public double getCurrentPosition() {
  //   return encoder.getPosition();
  // }

  public void setTargetPosition(double position) {
    targetPosition = position;
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
