/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Hood extends SubsystemBase {
  /**
   * Creates a new Hood.
   */
  VictorSPX hoodMotor;
  Encoder encoder;
  AnalogInput limit;
  public Hood() {
    hoodMotor = new VictorSPX(HOOD_MOTOR);
    encoder =  new Encoder(HOOD_ENCODER_A, HOOD_ENCODER_B);
    limit = new AnalogInput(HOOD_LIMIT);
  }

  public void resetEncoder(){
    encoder.reset();
  }

  public void recalibratePosition(){
    while(limit.getVoltage()<.4){
      hoodMotor.set(ControlMode.PercentOutput, HOOD_SPEED);
    }
      hoodMotor.set(ControlMode.PercentOutput, 0);
      resetEncoder();
  }

  public void moveToAngle(double angle){
    angle/360 * 42 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
