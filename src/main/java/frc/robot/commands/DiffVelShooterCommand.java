package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class DiffVelShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    public DiffVelShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.coast();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.brake();
    }

    @Override
    public boolean isFinished() {
    return false;
  }



}


