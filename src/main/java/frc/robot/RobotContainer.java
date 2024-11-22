// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.relation.RoleNotFoundException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RunSwerveDrive;
import frc.robot.subsystems.SwerveBase;

public class RobotContainer {
  private final SendableChooser<Command> autonChooser;
  private final SwerveBase drive = new SwerveBase();
  private PathPlannerAuto ovalAuton = new PathPlannerAuto("OvalTestAuto");
  private final CommandXboxController driver = new CommandXboxController(0);
  private final RunSwerveDrive runSwerveDrive = new RunSwerveDrive(driver,drive);
  public RobotContainer() {
    configureBindings();
    autonChooser = AutoBuilder.buildAutoChooser();
    autonChooser.setDefaultOption("Oval Auton", ovalAuton);
  }
  
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  private void configureBindings(){
    /* AMRIT AND TIA after setting up drivebase code from tuner x make sure to uncomment this and add the correct names, i just put standard names for now(from documentation)
    drive.setDefaultCommand( 
                drive.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed) 
                                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed) 
                                .withRotationalRate(-driver.getRightX() * RobotConstants.kMaxAngularRate) 
                ));*/
  }
  public void setDefaultCommands(){
    drive.setDefaultCommand(runSwerveDrive);
  }
  
}

