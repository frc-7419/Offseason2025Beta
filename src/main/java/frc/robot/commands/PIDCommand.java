package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class PIDCommand extends Command{

    PIDController topShooterPIDController;
    PIDController bottomShooterPIDController;

    ShooterSubsystem shooterSubsystem;

    double desiredRPM = ShooterConstants.desiredRPM;

    public PIDCommand(ShooterSubsystem shooterSubsystem){
        topShooterPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               shooterSubsystem.getTopEncoderVelocity());
        bottomShooterPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               shooterSubsystem.getBottomEncoderVelocity());

        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);

    }

    public void initialize() {
        topShooterPIDController.setTolerance(2);
        bottomShooterPIDController.setTolerance(2);

        topShooterPIDController.reset();
        bottomShooterPIDController.reset();
        
        topShooterPIDController.setSetpoint(convertToRPM(desiredRPM));
        bottomShooterPIDController.setSetpoint(convertToRPM(desiredRPM));

    }



    public void execute(){
        double PIDOutput = topShooterPIDController.calculate(shooterSubsystem.getTopEncoderPosition());
        shooterSubsystem.setVoltage(PIDOutput);
    }


    public double convertToRPM(double convertee) {
        double rpm = Units.RPM.convertFrom(convertee, Units.RPM);
        return rpm;
    }

    public double convertToRPS(double convertee) {
        double rps = Units.RevolutionsPerSecond.convertFrom(convertee, Units.RevolutionsPerSecond);
        return rps;
    }

    public double convertToMS(double convertee) {
        double ms = Units.MetersPerSecond.convertFrom(convertee, Units.MetersPerSecond);
        return ms;
    }

    
}
