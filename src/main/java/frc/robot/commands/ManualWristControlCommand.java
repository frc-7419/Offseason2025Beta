package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterWrist;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualWristControlCommand extends Command {
    private final ShooterWrist shooterWrist;
    private final CommandXboxController controller;
    private double speedMultiplier = 0.5; 
    private boolean isManualMode = true; // Start in manual mode

    public ManualWristControlCommand(ShooterWrist shooterWrist, CommandXboxController controller) { // Updated parameter type
        this.shooterWrist = shooterWrist;
        this.controller = controller;
        addRequirements(shooterWrist);
    }

    @Override
    public void execute() {
        double joystickInput = controller.getLeftY();
        shooterWrist.setPower(joystickInput * speedMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        shooterWrist.setPower(0); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
    }

    public void toggleMode() {
        isManualMode = !isManualMode;
    }
}