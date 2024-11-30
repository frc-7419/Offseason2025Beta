package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax topShooterMotors;
    private final SparkMax bottomShooterMotors;

    
    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private PIDController topShooterPIDController1 = null;
    private PIDController topShooterPIDController2 = null;
    private PIDController bottomShooterPIDController1 = null;
    private PIDController bottomShooterPIDController2 = null;

    public ShooterSubsystem() {
        topShooterMotors = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        bottomShooterMotors = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);

        topEncoder = topShooterMotors.getEncoder();
        bottomEncoder = bottomShooterMotors.getEncoder();
        
        topShooterPIDController1.setTolerance(2);
        topShooterPIDController1.setSetpoint(ShooterConstants.desiredRPM);

        topShooterPIDController2.setTolerance(2);
        topShooterPIDController2.setSetpoint(ShooterConstants.desiredRPM);

        bottomShooterPIDController1.setTolerance(2);
        bottomShooterPIDController1.setSetpoint(ShooterConstants.desiredRPM);

        bottomShooterPIDController2.setTolerance(2);
        bottomShooterPIDController2.setSetpoint(ShooterConstants.desiredRPM);
    }

    public void coast() {
        topShooterMotors.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        bottomShooterMotors.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    }

    public void brake() {
        topShooterMotors.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        bottomShooterMotors.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    }

    // Set PID coefficients from ShooterConstants
    private void configurePIDControllers() {
        RelativeEncoder topEncoder = topShooterMotors.getEncoder();
        RelativeEncoder bottomEncoder = bottomShooterMotors.getEncoder();
        
        topShooterPIDController1.setP(ShooterConstants.kP);
        topShooterPIDController1.setI(ShooterConstants.kI);
        topShooterPIDController1.setD(ShooterConstants.kD);

        topShooterPIDController2.setP(ShooterConstants.kP);
        topShooterPIDController2.setI(ShooterConstants.kI);
        topShooterPIDController2.setD(ShooterConstants.kD);

        bottomShooterPIDController1.setP(ShooterConstants.kP);
        bottomShooterPIDController1.setI(ShooterConstants.kI);
        bottomShooterPIDController1.setD(ShooterConstants.kD);

        bottomShooterPIDController2.setP(ShooterConstants.kP);
        bottomShooterPIDController2.setI(ShooterConstants.kI);
        bottomShooterPIDController2.setD(ShooterConstants.kD);

        topShooterPIDController1 = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               topEncoder.getVelocity());
        topShooterPIDController2 = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               topEncoder.getVelocity());
        bottomShooterPIDController1 = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               bottomEncoder.getVelocity());
        bottomShooterPIDController2 = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
                                               bottomEncoder.getVelocity());
        
        topShooterPIDController1.setSetpoint(convertToRPM(ShooterConstants.desiredRPM));
        topShooterPIDController2.setSetpoint(convertToRPM(ShooterConstants.desiredRPM));
        bottomShooterPIDController1.setSetpoint(convertToRPM(ShooterConstants.desiredRPM));
        bottomShooterPIDController2.setSetpoint(convertToRPM(ShooterConstants.desiredRPM));
    }

    public void setPower(double power) {
        topShooterMotors.set(power);
        bottomShooterMotors.set(power);
    }

    public void setvoltage(double voltage) {
        topShooterMotors.setVoltage(voltage);
        bottomShooterMotors.setVoltage(voltage);
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

    @Override
    public void periodic() {
        // Get current draw
        SmartDashboard.putNumber("Top Shooter Motors' Current Draw", topShooterMotors.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motors' Current Draw", bottomShooterMotors.getOutputCurrent());

        // Get voltage
        SmartDashboard.putNumber("Top Shooter Motors' Voltage", topShooterMotors.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motors' Voltage", bottomShooterMotors.getBusVoltage());

        // Get speed
        SmartDashboard.putNumber("Top Shooter Motors' Speed", topShooterMotors.get());
        SmartDashboard.putNumber("Bottom Shooter Motors' Speed", bottomShooterMotors.get());

        // Get RPM
        SmartDashboard.putNumber("Top Shooter Motors' RPM", convertToRPM(topShooterMotors.get()));
        SmartDashboard.putNumber("Bottom Shooter Motors' RPM", convertToRPM(bottomShooterMotors.get()));

        // Get RPS
        SmartDashboard.putNumber("Top Shooter Motors' RPS", convertToRPS(topShooterMotors.get()));
        SmartDashboard.putNumber("Bottom Shooter Motors' RPS", convertToRPS(bottomShooterMotors.get()));

        // Get m/s
        SmartDashboard.putNumber("Top Shooter Motors' MS", convertToMS(topShooterMotors.get()));
        SmartDashboard.putNumber("Bottom Shooter Motors' MS", convertToMS(bottomShooterMotors.get()));

        // Get temperature
        SmartDashboard.putNumber("Top Shooter Motors' Temperature", topShooterMotors.getMotorTemperature());
        SmartDashboard.putNumber("Bottom Shooter Motors' Temperature", bottomShooterMotors.getMotorTemperature());
    }

    public double get() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }
}