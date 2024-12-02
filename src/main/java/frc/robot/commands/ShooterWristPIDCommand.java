// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.ShooterWrist;

public class ShooterWristPIDCommand extends Command {
    private final ShooterWrist shooterWrist;
    private final ProfiledPIDController shooterWristPIDController;
    private final ArmFeedforward armFeedforward;
    private final double setPoint;

    public ShooterWristPIDCommand(ShooterWrist shooterWrist, double setPoint) {
        this.shooterWrist = shooterWrist;
        this.setPoint = setPoint;
        this.armFeedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kA);
        this.shooterWristPIDController = new ProfiledPIDController(
            WristConstants.kP, 
            WristConstants.kI, 
            WristConstants.kD, 
            new TrapezoidProfile.Constraints(WristConstants.maxVelocity, WristConstants.maxAcceleration)
        );
        addRequirements(shooterWrist);
    }

    @Override
    public void initialize() {
        shooterWrist.coast();
        shooterWristPIDController.setTolerance(0.5);
        shooterWristPIDController.reset(shooterWrist.getPosition().in(Units.Radians));
        shooterWristPIDController.setGoal(setPoint);
    }

    @Override
    public void execute() {
        Angle currentPosition = shooterWrist.getPosition();
        AngularVelocity currentVelocity = shooterWrist.getVelocityInRadians();
        double pidOutput = shooterWristPIDController.calculate(currentPosition.in(Units.Radian));
        double feedforwardOutput = armFeedforward.calculate(currentPosition.in(Units.Radians), currentVelocity.in(Units.RadiansPerSecond));
        shooterWrist.setPower(pidOutput + feedforwardOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterWrist.setPower(0);
        shooterWrist.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooterWristPIDController.atGoal();
    }
}
  
