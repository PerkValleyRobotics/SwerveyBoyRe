// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCMD;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RetroSwerveLib.*;


public class RobotContainer {

  // subsystems
  SwerveSubsystem swerveDrive = new SwerveSubsystem();
  
  // controllers 
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.xboxPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // set the drive command as the default command 
    swerveDrive.setDefaultCommand(new SwerveDriveCMD(swerveDrive, () -> SwerveMath.deadBand(0.05, m_driverController.getLeftX()), () -> SwerveMath.deadBand(0.05, m_driverController.getLeftY()), () -> m_driverController.getRightX(), () -> m_driverController.getRightY()));


    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {
    
  }

  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}