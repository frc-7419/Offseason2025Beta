// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeWithController extends Command {
  private final CommandXboxController joystick;
  /** Creates a new RunIntakeWithController. */
  public RunIntakeWithController( CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((joystick.getRightBumperPressed || joystick.getLeftBumperPressed()));
      intakeSubsystem.setPower(0.9)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
