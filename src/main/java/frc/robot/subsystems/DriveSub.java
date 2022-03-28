// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSub extends SubsystemBase {
  // motors, motor groups, and drive type
  private final CANSparkMax m_zero = 
    new CANSparkMax(DriveConstants.kLeftMotor01CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_one =
    new CANSparkMax(DriveConstants.kLeftMotor02CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_two =
    new CANSparkMax(DriveConstants.kRightMotor03CanBusID, MotorType.kBrushless);
  private final CANSparkMax m_three =
    new CANSparkMax(DriveConstants.kRightMotor04CanBusID, MotorType.kBrushless);
    private final Spark flagMotor = new Spark(0);

  private final MotorControllerGroup m_LeftMotors =
    new MotorControllerGroup(m_zero, m_one);
  private final MotorControllerGroup m_RightMotors =
    new MotorControllerGroup(m_two, m_three);

  private final DifferentialDrive m_drive =
    new DifferentialDrive(m_LeftMotors, m_RightMotors);

  // Creates a new Drive Subsystem.
  public DriveSub() {
    // invert left side
    m_LeftMotors.setInverted(true);
    
    // set followers
    m_one.follow(m_zero);
    m_two.follow(m_three);

    m_drive.setMaxOutput(DriveConstants.kMaxDrive);
    m_drive.setDeadband(DriveConstants.kDeadBand);

    // set how many seconds to reach maximum output
    m_zero.setOpenLoopRampRate(DriveConstants.kRampRate);
    m_one.setOpenLoopRampRate(DriveConstants.kRampRate);
    m_two.setOpenLoopRampRate(DriveConstants.kRampRate);
    m_three.setOpenLoopRampRate(DriveConstants.kRampRate);
  }

  // drive method is arcade
  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(-fwd*Math.abs(fwd), rot);
  }

  // individual motors
  public void runZero(double power){
    m_zero.set(power);
  }
  public void runOne(double power){
    m_one.set(power);
  }
  public void runTwo(double power){
    m_two.set(power);
  }
  public void runThree(double power){
    m_three.set(power);
  }
  public void flag(double power){
    flagMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
