// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain INSTANCE;

  private final CANSparkMax leftMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(6, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();
  private final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  public final AHRS gyro = new AHRS();
  

  private final DifferentialDriveOdometry odometry;
  private Drivetrain() {
    leftMotor1.setInverted(false);
    rightMotor1.setInverted(true);
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
    zeroEncoder();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void setVoltages(double left, double right) {
    leftMotor1.setVoltage(left);
    rightMotor1.setVoltage(right);
  }

  public void stop() {
    setVoltages(0, 0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    setVoltages(leftVolts, rightVolts);
    // backRightMotor.setVoltage(-rightVolts);
    // backLeftMotor.setVoltage(leftVolts);
  }

  public void arcadeDrive(double throttle, double twist, boolean squareInputs) {
    arcadeDrive(throttle, twist, squareInputs, 0.05);
  }
  
  public void arcadeDrive(double throttle, double twist, boolean squareInputs, double deadband) {
    if (squareInputs) {
      throttle *= Math.abs(throttle);
      twist *= Math.abs(twist);
    }
    if (Math.abs(throttle) < deadband) {
      throttle = 0;
    }
    if (Math.abs(twist) < deadband) {
      twist = 0;
    }

    setVoltages(10 * (throttle - twist), 10 * (throttle + twist));
  }

  public double getLeftEncoderRevs() {
    return (leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2.0;
  }

  public double getLeftEncoderDistance() {
    return getLeftEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }

  public double getRightEncoderRevs() {
    return (rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0;
  }

  public double getRightEncoderDistance() {
    return getRightEncoderRevs() * WHEEL_CIRCUMFRENCE / GEAR_RATIO;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder1.getVelocity()/60 * WHEEL_CIRCUMFRENCE / GEAR_RATIO, 
        rightEncoder1.getVelocity() / 60 * WHEEL_CIRCUMFRENCE / GEAR_RATIO);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    zeroEncoder();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void zeroEncoder() {
    leftEncoder1.setPosition(0);
    leftEncoder2.setPosition(0);
    rightEncoder1.setPosition(0);
    rightEncoder2.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    System.out.println(getLeftEncoderDistance() + ", " + getRightEncoderDistance() + ", " + odometry.getPoseMeters());
  }

  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }
}
