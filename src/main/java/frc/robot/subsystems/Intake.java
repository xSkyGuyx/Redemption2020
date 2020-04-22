/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.*;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  VictorSPX intakeMotor;
  boolean running = false;
  boolean intakeOut = false;
  DoubleSolenoid deployPiston;
  public Intake() {
    intakeMotor = new VictorSPX(INTAKE_MOTOR);
    setDefaultCommand(intakeStop);
    deployPiston = new DoubleSolenoid(INTAKE_SOLENOID_A, INTAKE_SOLENOID_B);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Running", running);
    SmartDashboard.putBoolean("Intake Deployed", intakeOut);
    // This method will be called once per scheduler run
  }

  public void intakeForward(){
    intakeMotor.set(ControlMode.PercentOutput, INTAKE_FORWARD_SPEED);
    running = true;
  }

  public void intakeReverse(){
    intakeMotor.set(ControlMode.PercentOutput, INTAKE_REVERSE_SPEED);
    running = true;
  }

  public void intakeStop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
    running = false;
  }

  public void deployIntake(){
    deployPiston.set(Value.kForward);
    intakeOut = true;
  }

  public void recallIntake(){
    deployPiston.set(Value.kReverse);
    intakeOut = false;
  }

}
