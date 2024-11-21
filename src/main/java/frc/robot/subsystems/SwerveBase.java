// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveBase extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule rearLeft;  
  private final SwerveModule rearRight;
  private final Pigeon2 gyro;

  public SwerveBase() {
    frontLeft = new SwerveModule(0, 1, "Front Left");
    frontRight = new SwerveModule(2, 3, "Front Right");
    rearLeft = new SwerveModule(4, 5, "Rear Left");
    rearRight = new SwerveModule(6, 7, "Rear Right");
    gyro = new Pigeon2(8);
  }
  public void setSwerveState(SwerveModuleState[] states){
    frontLeft.setSwerveState(states[0]);
    frontRight.setSwerveState(states[1]);
    rearLeft.setSwerveState(states[2]);
    rearRight.setSwerveState(states[3]);
  }
  public void setSwerveWithChassis(ChassisSpeeds chassis){
    setSwerveState(new SwerveDriveKinematics().toSwerveModuleStates(chassis));
  }
  public ChassisSpeeds getChassisSpeedsFromController(double vx, double vy, double rx){
    vx = Math.abs(vx)>0.1 ? vx : 0;
    vy = Math.abs(vy)>0.1 ? vy : 0;
    rx = Math.abs(rx)>0.1 ? rx : 0;
    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, gyro.getRotation2d());
  }
  public void coast(){
    frontLeft.coast();
    frontRight.coast();
    rearLeft.coast();
    rearRight.coast();  
  }
  public void brake(){
    frontLeft.brake();
    frontRight.brake();
    rearLeft.brake();
    rearRight.brake();  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
