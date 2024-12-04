// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.commands.ShooterWristPIDCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.WristConstants;

public class RobotContainer {
  private final ShooterWrist wrist = new ShooterWrist();
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
  }

  private void configureBindings() {
    // Bind wrist positions to buttons
    operator.a().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.scoring));
    operator.b().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.intakePosition));
    operator.x().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.homePosition));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void setDefaultCommands() {
    
  }
}
