// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.NetworkInterface;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // Creates the Drive subsystem
  private final DriveSub m_Drive = new DriveSub();
  // Creates the connection to the xBox controller
  XboxController m_xBox = new XboxController(OIConstants.kDriveControllerPort);
  // finds the SmartDashboard tab of Shuffleboard
  private final ShuffleboardTab sbStuffs = Shuffleboard.getTab("myStuffs");

  // network tables stuffs
  private final NetworkTable m_ntVariables = NetworkTableInstance.getDefault().getTable("default");
  private final NetworkTableEntry flagSpeed = sbStuffs.add("flag speed", 0.0).getEntry();


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_Drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //    m_robotDrive
    //.setDefaultCommand(new RunCommand(() -> m_robotDrive.arcadeDrive(-m_xboxController.getRightY(),
    //-m_xboxController.getLeftX()), m_robotDrive));
    m_Drive.setDefaultCommand(new RunCommand(() -> 
      m_Drive.arcadeDrive(-m_xBox.getRightY(), -m_xBox.getLeftX()), m_Drive));

    // playing with Shuffleboard
    // SmartDashboard.putNumber("test", 0.5);
    //sbSmart.add("test value", 0.5);
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton runThreeButton = new JoystickButton(m_xBox, OIConstants.kTestAButton);
    runThreeButton
      .whileHeld(new RunCommand(
        () -> m_Drive.flag(flagSpeed.getDouble(0.2)  ),m_Drive))
      .whenReleased(new RunCommand(
        () -> m_Drive.flag(0.0),m_Drive));
/*
    final JoystickButton updateTables = new JoystickButton(m_xBox, OIConstants.kTestBButton);
    updateTables.whenPressed
  */  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
