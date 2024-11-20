// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Beambrake extends SubsystemBase {
  /** Creates a new Beambrake. */
  private final DigitalInput beamBreakFront;
  private final DigitalInput beamBreakBack;
  public Beambrake() {
    beamBreakFront = new DigitalInput(2);
    beamBreakBack = new DigitalInput(7);
  }
  public boolean frontBeamBreakIsTriggered() {
    return !beamBreakFront.get();
  }

  public boolean backBeamBreakIsTriggered() {
    return !beamBreakBack.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Front Beam Break Triggered",  frontBeamBreakIsTriggered());
    SmartDashboard.putBoolean("Back Beam Brake Triggered", backBeamBreakIsTriggered());
  }
}
