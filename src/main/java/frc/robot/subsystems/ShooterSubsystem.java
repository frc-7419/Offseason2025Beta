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
    SparkMax topShooterMotor;
    SparkMax bottomShooterMotor;

    
    RelativeEncoder topEncoder;
    RelativeEncoder bottomEncoder;



    double desiredRPM = ShooterConstants.desiredRPM;

    public ShooterSubsystem() {
        topShooterMotor = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        bottomShooterMotor = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);
        

    }

    public void coast() {
        topShooterMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        bottomShooterMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    }

    public void brake() {
        topShooterMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        bottomShooterMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    }

    public void setPower(double power) {
        topShooterMotor.set(power);
        bottomShooterMotor.set(power);
    }

    public void setVoltage(double voltage) {
        topShooterMotor.setVoltage(voltage);
        bottomShooterMotor.setVoltage(voltage);
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

    public double getTopEncoderVelocity() {
        return topEncoder.getVelocity();
    }

    public double getBottomEncoderVelocity() {
        return bottomEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        // Get current draw
        SmartDashboard.putNumber("Top Shooter Motor Current Draw", topShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motor Current Draw", bottomShooterMotor.getOutputCurrent());

        // Get voltage
        SmartDashboard.putNumber("Top Shooter Motor Voltage", topShooterMotor.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motor Voltage", bottomShooterMotor.getBusVoltage());

        // Get speed
        SmartDashboard.putNumber("Top Shooter Motor Speed", topShooterMotor.get());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Speed", bottomShooterMotor.get());

        // Get RPM
        SmartDashboard.putNumber("Top Shooter Motor RPM", convertToRPM(topShooterMotor.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor RPM", convertToRPM(bottomShooterMotor.get()));

        // Get RPS
        SmartDashboard.putNumber("Top Shooter Motor RPS", convertToRPS(topShooterMotor.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor RPS", convertToRPS(bottomShooterMotor.get()));

        // Get m/s
        SmartDashboard.putNumber("Top Shooter Motor MS", convertToMS(topShooterMotor.get()));
        SmartDashboard.putNumber("Bottom Shooter Motor MS", convertToMS(bottomShooterMotor.get()));

        // Get temperature
        SmartDashboard.putNumber("Top Shooter Motor Temperature", topShooterMotor.getMotorTemperature());
        SmartDashboard.putNumber("Bottom Shooter Motor Temperature", bottomShooterMotor.getMotorTemperature());
    }

    public double get() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }
}