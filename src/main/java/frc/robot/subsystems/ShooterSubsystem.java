package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    SparkMax topShooterMotor1;
    SparkMax topShooterMotor2;
    SparkMax bottomShooterMotor1;
    SparkMax bottomShooterMotor2;

    PIDController topShooterPIDController1;
    PIDController topShooterPIDController2;
    PIDController bottomShooterPIDController1;
    PIDController bottomShooterPIDController2;

    double desiredRPM = ShooterConstants.desiredRPM;

    public ShooterSubsystem() {
        topShooterMotor1 = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        topShooterMotor2 = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        bottomShooterMotor1 = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);
        bottomShooterMotor2 = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);
        
        topShooterPIDController1.setTolerance(2);
        topShooterPIDController1.setSetpoint(desiredRPM);
    }

    public void coast() {
        topShooterMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        topShooterMotor2.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        bottomShooterMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        bottomShooterMotor2.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    }

    public void brake() {
        topShooterMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        topShooterMotor2.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        bottomShooterMotor1.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        bottomShooterMotor2.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    }

    // Set PID coefficients from ShooterConstants
    private void configurePIDControllers() {
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
    }

    public void setPower(double power) {
        topShooterMotor1.set(power);
        topShooterMotor2.set (power);
        bottomShooterMotor1.set(power);
        bottomShooterMotor2.set (power);
    }

    public void setvoltage(double voltage) {
        topShooterMotor1.setVoltage(voltage);
        topShooterMotor2.setVoltage(voltage);
        bottomShooterMotor1.setVoltage(voltage);
        bottomShooterMotor2.setVoltage(voltage);
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
        SmartDashboard.putNumber("Top Shooter Motor 1 Current Draw", topShooterMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Top Shooter Motor 2 Current Draw", topShooterMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Current Draw", bottomShooterMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Current Draw", bottomShooterMotor2.getOutputCurrent());

        // Get voltage
        SmartDashboard.putNumber("Top Shooter Motor 1 Voltage", topShooterMotor1.getBusVoltage());
        SmartDashboard.putNumber("Top Shooter Motor 2 Voltage", topShooterMotor2.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Voltage", bottomShooterMotor1.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Voltage", bottomShooterMotor2.getBusVoltage());

        // Get speed
        SmartDashboard.putNumber("Top Shooter Motor 1 Speed", topShooterMotor1.get());
        SmartDashboard.putNumber("Top Shooter Motor 2 Speed", topShooterMotor2.get());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Speed", bottomShooterMotor1.get());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Speed", bottomShooterMotor2.get());

        // Get RPM
        SmartDashboard.putNumber("Top Shooter Motor 1 RPM", convertToRPM(topShooterMotor1.get()));
        SmartDashboard.putNumber("Top Shooter Motor 2 RPM", convertToRPM(topShooterMotor2.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 1 RPM", convertToRPM(bottomShooterMotor1.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 2 RPM", convertToRPM(bottomShooterMotor2.get()));

        // Get RPS
        SmartDashboard.putNumber("Top Shooter Motor 1 RPS", convertToRPS(topShooterMotor1.get()));
        SmartDashboard.putNumber("Top Shooter Motor 2 RPS", convertToRPS(topShooterMotor2.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 1 RPS", convertToRPS(bottomShooterMotor1.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 2 RPS", convertToRPS(bottomShooterMotor2.get()));

        // Get m/s
        SmartDashboard.putNumber("Top Shooter Motor 1 MS", convertToMS(topShooterMotor1.get()));
        SmartDashboard.putNumber("Top Shooter Motor 2 MS", convertToMS(topShooterMotor2.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 1 MS", convertToMS(bottomShooterMotor1.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor 2 MS", convertToMS(bottomShooterMotor2.get()));

        // Get temperature
        SmartDashboard.putNumber("Top Shooter Motor 1 Temperature", topShooterMotor1.getMotorTemperature());
        SmartDashboard.putNumber("Top Shooter Motor 2 Temperature", topShooterMotor2.getMotorTemperature());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Temperature", bottomShooterMotor1.getMotorTemperature());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Temperature", bottomShooterMotor2.getMotorTemperature());
    }

    public double get() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }
}