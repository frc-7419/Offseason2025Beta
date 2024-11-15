// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.ShooterWrist;

public class RobotContainer {
  private final ShooterWrist m_wrist = new ShooterWrist();
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Bind wrist positions to buttons
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
