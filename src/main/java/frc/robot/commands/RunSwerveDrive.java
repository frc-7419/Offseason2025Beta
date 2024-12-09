// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunSwerveDrive extends Command {
  private final SwerveBase swerveBase;
  private final CommandXboxController driver;
  public RunSwerveDrive(CommandXboxController driver, SwerveBase swerveBase) {
    this.swerveBase = swerveBase;
    this.driver = driver;
    addRequirements(swerveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveBase.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveBase.setSwerveWithChassis(swerveBase.getChassisSpeedsFromController(driver.getLeftY(), driver.getLeftX(), driver.getRightX()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveBase.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
