// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.ShooterWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterWristCommand extends Command {
  /** Creates a new ShooterWristCommand. */
  private final ShooterWrist shooterWrist;
  private final PIDController shooterWristPIDController;
  private final double feedForward = (0.9 / 12) / 2.67;
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.02809 * 2.5, 0.01 * 1.5);
  private final double setPoint;

  public ShooterWristCommand(ShooterWrist shooterWrist, double setPoint, double feedForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterWrist = shooterWrist;
    this.setPoint = setPoint;
    this.feedForward = feedForward;
    this.shooterWristPIDController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD );
    addRequirements(shooterWrist);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWristPIDController.setTolerance(0.5);
    shooterWristPIDController.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = shooterWrist.getCurrentPosition();
    double currentVelocity = shooterWrist.getVelocityInRadians();
    shooterWrist.setPower(armFeedforward.calculate(currentPosition, currentVelocity) + shooterWristPIDController.calculate(setPoint));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.brake();
    shooterWrist.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  }

