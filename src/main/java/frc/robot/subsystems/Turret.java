/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  VictorSPX turretMotor;
  boolean moving = false;
  public Turret() {
    turretMotor = new VictorSPX(TURRET_MOTOR);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Turret Moving", moving);
    // This method will be called once per scheduler run
  }

  public void moveTurret(double speed){
    turretMotor.set(ControlMode.PercentOutput, speed);
    moving = true;
  }

  public void stopTurret(){
    turretMotor.set(ControlMode.PercentOutput, 0);
    moving = false;
  }
}
