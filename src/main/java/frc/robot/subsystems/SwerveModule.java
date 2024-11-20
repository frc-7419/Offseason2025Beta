// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final TalonFX turnMotor;
  private final TalonFX driveMotor;
  private final String moduleName;
  private final PIDController angleController;
  public SwerveModule(int turnMotorID, int driveMotorID, String moduleName) {
    turnMotor = new TalonFX(turnMotorID);
    driveMotor = new TalonFX(driveMotorID);
    this.moduleName = moduleName;
    angleController = new PIDController(2, 0, 1);
  }
  public void setSwerveState(SwerveModuleState state){
    //the optimize function is apparently deprecated
    driveMotor.set(state.speedMetersPerSecond);
    angleController.setSetpoint(state.angle.getDegrees());
    turnMotor.set(angleController.calculate(turnMotor.getPosition().getValueAsDouble()));
  }
  public void brake(){
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void coast(){
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    turnMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
