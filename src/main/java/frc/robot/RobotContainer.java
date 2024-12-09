package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualWristControlCommand;
import frc.robot.commands.RunSwerveDrive;
import frc.robot.commands.ShooterWristPIDCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final SendableChooser<Command> autonChooser;
    private final SwerveBase drive = new SwerveBase();
    private final ShooterWrist wrist = new ShooterWrist();
    private final ShooterWristPIDCommand shooterWristPIDCommand = new ShooterWristPIDCommand(wrist, WristConstants.scoring);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final RunSwerveDrive runSwerveDrive = new RunSwerveDrive(driver, drive);
    private PathPlannerAuto ovalAuton = new PathPlannerAuto("OvalTestAuto");

    public RobotContainer() {
        configureBindings();
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Current auton", autonChooser);
        setDefaultCommands();
    }

    private void configureBindings() {
        // Bind wrist positions to buttons
        operator.a().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.scoring));
        operator.b().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.intakePosition));
        operator.x().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.homePosition));
        
        operator.y().whileTrue(new ManualWristControlCommand(wrist, operator));
        
        operator.povUp().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(1.0))); // Full speed
        operator.povDown().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(0.25))); // Reduced speed
        operator.start().onTrue(new InstantCommand(() -> manualWristControlCommand.toggleMode()));

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualWristControlCommand;
import frc.robot.commands.RunSwerveDrive;
import frc.robot.commands.ShooterWristPIDCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final SendableChooser<Command> autonChooser;
    private final SwerveBase drive = new SwerveBase();
    private final ShooterWrist wrist = new ShooterWrist();
    private final ShooterWristPIDCommand shooterWristPIDCommand = new ShooterWristPIDCommand(wrist, WristConstants.scoring);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final RunSwerveDrive runSwerveDrive = new RunSwerveDrive(driver, drive);
    private PathPlannerAuto ovalAuton = new PathPlannerAuto("OvalTestAuto");

    public RobotContainer() {
        configureBindings();
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Current auton", autonChooser);
        setDefaultCommands();
    }

    private void configureBindings() {
        // Bind wrist positions to buttons
        operator.a().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.scoring));
        operator.b().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.intakePosition));
        operator.x().onTrue(new ShooterWristPIDCommand(wrist, WristConstants.homePosition));
        
        operator.y().whileTrue(new ManualWristControlCommand(wrist, operator));
        
        operator.povUp().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(1.0))); // Full speed
        operator.povDown().onTrue(new InstantCommand(() -> manualWristControlCommand.setSpeedMultiplier(0.25))); // Reduced speed
        operator.start().onTrue(new InstantCommand(() -> manualWristControlCommand.toggleMode()));

        /*
        AMRIT AND TIA after setting up drivebase code from tuner x make sure to uncomment this and add the correct names, i just put standard names for now (from documentation)
        drive.setDefaultCommand(
            drive.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                .withRotationalRate(-driver.getRightX() * RobotConstants.kMaxAngularRate)
            ));
        */
    }

    public Command getAutonomousCommand() {
        Command selectedAuton = autonChooser.getSelected();
        if (selectedAuton == null) {
            SmartDashboard.putString("Auton Status", "No autonomous command selected!");
            System.out.println("No autonomous command selected");
            return Commands.none(); 
        }
        SmartDashboard.putString("Auton Status", "Autonomous command selected: " + selectedAuton.getName());
        return selectedAuton;
    }

    private void setDefaultCommands() {
        wrist.setDefaultCommand(shooterWristPIDCommand);
        drive.setDefaultCommand(runSwerveDrive);
    }

    public Command getAutonomousCommand() {
        Command selectedAuton = autonChooser.getSelected();
        if (selectedAuton == null) {
            SmartDashboard.putString("Auton Status", "No autonomous command selected!");
            System.out.println("No autonomous command selected");
            return Commands.none(); 
        }
        SmartDashboard.putString("Auton Status", "Autonomous command selected: " + selectedAuton.getName());
        return selectedAuton;
    }

    private void setDefaultCommands() {
        wrist.setDefaultCommand(shooterWristPIDCommand);
        drive.setDefaultCommand(runSwerveDrive);
    }
}