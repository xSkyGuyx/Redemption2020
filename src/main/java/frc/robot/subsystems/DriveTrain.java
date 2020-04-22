/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private final CANSparkMax rightMaster, leftMaster;
  private final CANSparkMax rightSlaveBack, leftSlaveBack;
  private final CANSparkMax rightSlaveFront, leftSlaveFront;

  DoubleSolenoid speedShiftSol, elevatorShiftSol;
  
  Pose2d pose;

  ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TREAD_WIDTH_INCHES));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS_DRIVE, KV_DRIVE, KA_DRIVE);

  PIDController leftPIDController = new PIDController(KP_DRIVE, KI_DRIVE, KD_DRIVE);
  PIDController rightPIDController = new PIDController(KP_DRIVE, KI_DRIVE, KD_DRIVE);

  DifferentialDrive myRobot;

  /*
    * Creates a new DriveTrain.
  */
  public DriveTrain() {
    //Master Motors 
    rightMaster = new CANSparkMax(RIGHT_MASTER_DRIVE, MotorType.kBrushless);
    rightMaster.restoreFactoryDefaults();
    rightMaster.setInverted(false);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightMaster.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE);
    rightMaster.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
    leftMaster = new CANSparkMax(LEFT_MASTER_DRIVE, MotorType.kBrushless);
    leftMaster.restoreFactoryDefaults();
    leftMaster.setInverted(false);
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftMaster.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE);
    leftMaster.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);

    rightSlaveBack = new CANSparkMax(RIGHT_BACK_DRIVE, MotorType.kBrushless);
    rightSlaveBack.restoreFactoryDefaults();
    rightSlaveBack.follow(rightMaster, false);
    leftSlaveBack = new CANSparkMax(LEFT_BACK_DRIVE, MotorType.kBrushless);
    leftSlaveBack.restoreFactoryDefaults();
    leftSlaveBack.follow(leftMaster, false);

    rightSlaveFront = new CANSparkMax(RIGHT_FRONT_DRIVE, MotorType.kBrushless);
    rightSlaveFront.restoreFactoryDefaults();
    rightSlaveFront.follow(rightMaster, false);
    leftSlaveFront = new CANSparkMax(LEFT_FRONT_DRIVE, MotorType.kBrushless);
    leftSlaveFront.restoreFactoryDefaults();
    leftSlaveFront.follow(leftMaster, false);

    speedShiftSol = new DoubleSolenoid(SPEED_DRIVE_SHIFT_A, SPEED_DRIVE_SHIFT_B);
    elevatorShiftSol = new DoubleSolenoid(ELEVATOR_DRIVE_SHIFT_A, ELEVATOR_DRIVE_SHIFT_B);

    myRobot = new DifferentialDrive(leftMaster, rightMaster);

    pose = new Pose2d();
    
    gyro.reset();

    setDefaultCommand(defaultDrive);
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getPose(){
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS_INCHES) / 60,
        rightMaster.getEncoder().getVelocity() / GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS_INCHES) / 60
    );
  }

  public double getLeftDistanceMeters(){
    return Units.inchesToMeters(leftMaster.getEncoder().getPosition()*WHEEL_RADIUS_INCHES*2*Math.PI);
  }

  public double getRightDistanceMeters(){
    return Units.inchesToMeters(rightMaster.getEncoder().getPosition()*WHEEL_RADIUS_INCHES*2*Math.PI);
  }

  public void resetEncoders(){
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  /*
  @param leftVolts
  @param rightVolts 
  */
  public void setOutputVolts(final double leftVolts, final double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  public void stop(){
    leftMaster.set(0);
    rightMaster.set(0);
  }

  public void toggleGear(){
    if(isHighGear()){
      shiftToLow();
    }else{
      shiftToHigh();
    }
  }

  public void shiftToHigh(){
    if(leftMaster.getEncoder().getVelocity() < TOLERABLE_SHIFT_VELOCITY_RPM && rightMaster.getEncoder().getVelocity() < TOLERABLE_SHIFT_VELOCITY_RPM){
      speedShiftSol.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void shiftToLow(){
    if(leftMaster.getEncoder().getVelocity() < TOLERABLE_SHIFT_VELOCITY_RPM && rightMaster.getEncoder().getVelocity() < TOLERABLE_SHIFT_VELOCITY_RPM){
      speedShiftSol.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public boolean isHighGear(){
    return speedShiftSol.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean isElevator(){
    return elevatorShiftSol.get() == DoubleSolenoid.Value.kForward;
  }

  /*
  @params number
  */
  public double normalize(double number){
    if(number > 1.0){
      number = 1;
    }else if(number<-1.0){
      number = -1;
    }
    if(Math.abs(number)>CONTROLLER_DEADBAND){
      number = 0;
    }
    return number;
  }

  public void teleopDrive(double left, double right){
    left = normalize(left);
    right = normalize(right);
    if(left==0 && right!=0){
      curvature_drive(left, right, true);
    }else{
      curvature_drive(left, right, false);
    }
  }

  private void curvature_drive(double xSpeed, double zRotation, boolean isQuickTurn){
    xSpeed = normalize(xSpeed);

    zRotation = normalize(zRotation);

    myRobot.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Side Speed RPM", leftMaster.getEncoder().getCountsPerRevolution());
    SmartDashboard.putNumber("Right Side Speed RPM", rightMaster.getEncoder().getCountsPerRevolution());
    SmartDashboard.putBoolean("Is High Gear", isHighGear());
    SmartDashboard.putBoolean("Elevator Engaged", isElevator());
    pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    resetEncoders();
  }
}
