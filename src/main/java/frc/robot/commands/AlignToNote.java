// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToNote extends Command {
  private final VisionSubsystem vision;
  private final PIDController pidController;

  /** Creates a new AlignToNote. */
  public AlignToNote(VisionSubsystem vision) {
    this.vision = vision;
    pidController = new PIDController(0, 0, 0); // arbitrary values for now
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed = -pidController.calculate(vision.getNotePose().getRotation().getDegrees(),0); 
 
    rotationSpeed = Math.max(-0.5, Math.min(0.5,rotationSpeed)); // Clamp

    // drivetrain.setModuleStates(drivetrain.getChassisSpeedsFromJoystick(0,0,rotationSpeed,false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
