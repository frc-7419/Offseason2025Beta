// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.commands.ShooterWristMoveCommand;

public class RobotContainer {
  private final ShooterWrist wrist = new ShooterWrist();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final ShooterWristMoveCommand shooterWristMoveCommand;

  public RobotContainer() {
    shooterWristMoveCommand = new ShooterWristMoveCommand(wrist, driverController);
    configureBindings();
  }

  private void configureBindings() {
    // Bind wrist positions to buttons
    driverController.a().onTrue(shooterWristMoveCommand);
    driverController.b().onTrue(shooterWristMoveCommand);
    driverController.x().onTrue(shooterWristMoveCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
