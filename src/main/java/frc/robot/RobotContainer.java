// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.commands.ShooterWristMoveCommand;

public class RobotContainer {
  private final ShooterWrist m_wrist = new ShooterWrist();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final ShooterWristMoveCommand m_shooterWristMoveCommand;

  public RobotContainer() {
    m_shooterWristMoveCommand = new ShooterWristMoveCommand(m_wrist, m_driverController);
    configureBindings();
  }

  private void configureBindings() {
    // Bind wrist positions to buttons
    m_driverController.a().onTrue(m_shooterWristMoveCommand);
    m_driverController.b().onTrue(m_shooterWristMoveCommand);
    m_driverController.x().onTrue(m_shooterWristMoveCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
