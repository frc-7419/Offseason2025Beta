// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterWristMoveCommand extends Command {
  /** Creates a new ShooterWristMoveCommand. */
  private final ShooterWrist shooterWrist;
  private final CommandXboxController controller;
  public ShooterWristMoveCommand(ShooterWrist shooterWrist, CommandXboxController controller) {
    this.shooterWrist = shooterWrist;
    this.controller = controller;
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get joystick input for manual control
    double joystickInput = controller.getLeftY();

    shooterWrist.setPower(joystickInput);

    if (controller.a().getAsBoolean()) { 
      shooterWrist.setTargetPosition(WristConstants.HOME_POSITION);
    } else if (controller.b().getAsBoolean()) { 
      shooterWrist.setTargetPosition(WristConstants.SCORING);
    } else if (controller.x().getAsBoolean()) { 
      shooterWrist.setTargetPosition(WristConstants.INTAKE_POSITION);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
