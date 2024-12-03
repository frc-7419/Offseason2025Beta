// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeWithController extends Command {
  private final CommandXboxController joystick;
  private final IntakeSubsystem intakeSubsystem;
  private final BeamBreakSubsystem beamBreakSubsystem;

  public RunIntakeWithController(CommandXboxController joystick, IntakeSubsystem intakeSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.beamBreakSubsystem = beamBreakSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
    addRequirements(beamBreakSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 
    if ((joystick.rightBumper().getAsBoolean() || joystick.leftBumper().getAsBoolean())){ // Need to check if it return True when pressed or vice versa

      intakeSubsystem.setPower(0.9);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (beamBreakSubsystem.frontBeamBreakIsTriggered() == true || beamBreakSubsystem.backBeamBreakIsTriggered() == true) {
      intakeSubsystem.brake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

