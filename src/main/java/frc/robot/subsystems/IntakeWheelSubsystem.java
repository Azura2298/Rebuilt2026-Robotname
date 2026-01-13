// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeWheelSubsystem extends SubsystemBase {

  private SparkMax IntakeWheel =
      new SparkMax(RobotConstants.kIntakeWheelCanId, MotorType.kBrushless);

  /** Creates a new IntakeWheelSubsystem. */
  public IntakeWheelSubsystem() {}

  public void IntakeWheelForwardCmd() {
    IntakeWheel.set(RobotConstants.kIntakeWheelSpeed);
  }

  public void IntakeWheelReverseCmd() {
    IntakeWheel.set(-1 * RobotConstants.kIntakeWheelSpeed);
  }

  public void IntakeWheelStopCmd() {
    IntakeWheel.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
