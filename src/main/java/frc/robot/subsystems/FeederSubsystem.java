// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class FeederSubsystem extends SubsystemBase {

  private SparkMax feeder = new SparkMax(RobotConstants.kFeederCanId, MotorType.kBrushless);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {}

  public void feederForwardCmd() {
    feeder.set(RobotConstants.kFeederSpeed);
  }

  public void feederReverseCmd() {
    feeder.set(-1 * RobotConstants.kFeederSpeed);
  }

  public void feederStopCmd() {
    feeder.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
