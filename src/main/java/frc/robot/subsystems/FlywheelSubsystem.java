// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class FlywheelSubsystem extends SubsystemBase { // if a quiz is quizzical then what is a test

  private SparkMax flywheel = new SparkMax(RobotConstants.kFlywheelCanId, MotorType.kBrushless);

  public FlywheelSubsystem() {}

  public void flywheelForwardCmd() {
    flywheel.set(RobotConstants.kFlywheelSpeed);
  }

  public void flywheelStopCmd() {
    flywheel.stopMotor();
  }

  public void flywheelReverseCmd() {
    flywheel.set(-1 * RobotConstants.kFlywheelSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
