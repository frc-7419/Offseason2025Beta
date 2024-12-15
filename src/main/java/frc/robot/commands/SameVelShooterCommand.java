package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SameVelShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    public SameVelShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.coast();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        shooter.brake();
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}