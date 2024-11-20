package frc.robot.subsystems;

import frc.robot.constants.ShooterConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
        this.topShooterMotor1 = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        this.topShooterMotor2 = new SparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
        this.bottomShooterMotor1 = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);
        this.bottomShooterMotor2 = new SparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);
        
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Shooter Motor 1 Current", topShooterMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Top Shooter Motor 2 Current", topShooterMotor2.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Current", bottomShooterMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Current", bottomShooterMotor2.getOutputCurrent());

        SmartDashboard.putNumber("Top Shooter Motor 1 Voltage", topShooterMotor1.getBusVoltage());
        SmartDashboard.putNumber("Top Shooter Motor 2 Voltage", topShooterMotor2.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Voltage", bottomShooterMotor1.getBusVoltage());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Voltage", bottomShooterMotor2.getBusVoltage());

        SmartDashboard.putNumber("Top Shooter Motor 1 Speed", topShooterMotor1.get());
        SmartDashboard.putNumber("Top Shooter Motor 2 Speed", topShooterMotor2.get());
        SmartDashboard.putNumber("Bottom Shooter Motor 1 Speed", bottomShooterMotor1.get());
        SmartDashboard.putNumber("Bottom Shooter Motor 2 Speed", bottomShooterMotor2.get());
    }
}