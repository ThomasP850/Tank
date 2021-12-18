// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // Motors:
  private TalonSRX frMotor;
  private TalonSRX flMotor;
  private TalonSRX brMotor;
  private TalonSRX blMotor;

  private XboxController xbox;

  private DifferentialDriveKinematics kinematics;

  // Motor ids:
  public static final int FR_CAN_ID = 11;
  public static final int FL_CAN_ID = 10;
  public static final int BR_CAN_ID = 12;
  public static final int BL_CAN_ID = 13;

  public static final int CONTROLLER_PORT = 0;

  //TODO: measure actual distance
  // Distance across the front of the bot from center of wheel to center of wheel in meters.
  public static final double TRACKWIDTH = 1;


  @Override
  public void robotInit() {
    frMotor = new TalonSRX(FR_CAN_ID);
    flMotor = new TalonSRX(FL_CAN_ID);
    brMotor = new TalonSRX(BR_CAN_ID);
    blMotor = new TalonSRX(BL_CAN_ID);

    xbox = new XboxController(CONTROLLER_PORT);

    kinematics = new DifferentialDriveKinematics(TRACKWIDTH);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    ChassisSpeeds speeds = new ChassisSpeeds(xbox.getY(Hand.kLeft), 0, xbox.getX(Hand.kRight));
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.normalize(1);

    frMotor.set(TalonSRXControlMode.PercentOutput, wheelSpeeds.rightMetersPerSecond);
    flMotor.set(TalonSRXControlMode.PercentOutput, -wheelSpeeds.leftMetersPerSecond);

    brMotor.set(TalonSRXControlMode.Follower, FR_CAN_ID);
    blMotor.set(TalonSRXControlMode.Follower, FL_CAN_ID);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
