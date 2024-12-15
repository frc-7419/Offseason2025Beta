package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DiffVelShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final double kFTop; 
//BEFORE WORKING READ: still have to implement kFBottom & Do the same for Same VelShooter
    public DiffVelShooterCommand(ShooterSubsystem shooter, double kFTop) {
        this.shooter = shooter;
        this.kFTop = kFTop;
    }

    @Override
    public void initialize() {
        shooter.coast();
    }

    @Override
    public void execute() {
        // Fix to get Encoder Reading
        // double actualRPM = shooter.convertToRPM(shooter.get()); 
        double feedforwardOutput = (ShooterConstants.desiredRPM - shooter.convertToRPM(shooter.get())) * kFTop;
        shooter.setPower(feedforwardOutput);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.brake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}