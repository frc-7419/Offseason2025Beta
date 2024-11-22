// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunVelocity extends Command {
  /** Creates a new RunVelocity. */
  private IntakeSubsystem intakeSubsystem;
  private double reachVelocity;
  PIDController pidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  public RunVelocity(IntakeSubsystem intakeSubsystem, double reachVelocity) {
    this.intakeSubsystem = intakeSubsystem;
    this.reachVelocity = reachVelocity;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.coast();
    intakeSubsystem.setPower(0);
    pidController.setSetpoint(reachVelocity);
    pidController.setTolerance(5, 10); // Based off of old code 
  }

  // Called every time the scheduler runs while the command is schedule
  @Override
  public void execute() {
    double power = pidController.calculate(intakeSubsystem.getVelocity());
    intakeSubsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
