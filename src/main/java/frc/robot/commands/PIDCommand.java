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
        topShooterPIDController.setTolerance(2);
        topShooterPIDController.setSetpoint(desiredRPM);

        bottomShooterPIDController.setTolerance(2);
        bottomShooterPIDController.setSetpoint(desiredRPM);

        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);

    }

    private void configurePIDControllers() {
        topShooterPIDController.setP(ShooterConstants.kP);
        topShooterPIDController.setI(ShooterConstants.kI);
        topShooterPIDController.setD(ShooterConstants.kD);


        bottomShooterPIDController.setP(ShooterConstants.kP);
        bottomShooterPIDController.setI(ShooterConstants.kI);
        bottomShooterPIDController.setD(ShooterConstants.kD);


        topShooterPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               shooterSubsystem.getTopEncoderVelocity());
        bottomShooterPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               shooterSubsystem.getBottomEncoderVelocity());
        
        topShooterPIDController.setSetpoint(convertToRPM(desiredRPM));
        bottomShooterPIDController.setSetpoint(convertToRPM(desiredRPM));
    }

    public void execute(){
        double topShooterVelocity = shooterSubsystem.getTopEncoderVelocity();
        double bottomShooterVelocity = shooterSubsystem.getBottomEncoderVelocity();

       
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
