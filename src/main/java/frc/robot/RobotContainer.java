package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.commands.ShooterWristPIDCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.commands.ManualWristControlCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final ShooterWrist wrist = new ShooterWrist();
    private final ShooterWristPIDCommand ShooterWristPIDCommand = new ShooterWristPIDCommand(wrist, WristConstants.scoring);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);
    private final ManualWristControlCommand manualWristControlCommand = new ManualWristControlCommand(wrist, operator);

    public RobotContainer() {
        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {
        // Bind wrist positions to buttons
        operator.a().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.scoring));
        operator.b().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.intakePosition));
        operator.x().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.homePosition));
        
        operator.y().whileTrue(manualWristControlCommand);
        
        operator.povUp().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(1.0))); // Full speed
        operator.povDown().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(0.25))); // Reduced speed
        operator.start().onTrue(new InstantCommand(() -> manualWristControlCommand.toggleMode()));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    private void setDefaultCommands() {
        wrist.setDefaultCommand(ShooterWristPIDCommand);
    }
}
