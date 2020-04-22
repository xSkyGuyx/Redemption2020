/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Operator Interface
    public static final int DRIVER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = .15;
    //DriveTrain Falcons
    public static final int RIGHT_MASTER_DRIVE = 0;
    public static final int LEFT_MASTER_DRIVE = 3;
    public static final int RIGHT_BACK_DRIVE = 1;
    public static final int LEFT_BACK_DRIVE = 4;
    public static final int RIGHT_FRONT_DRIVE = 2;
    public static final int LEFT_FRONT_DRIVE = 5;
    public static final int SPEED_DRIVE_SHIFT_A = 0;
    public static final int SPEED_DRIVE_SHIFT_B = 1;
    public static final int ELEVATOR_DRIVE_SHIFT_A = 2;
    public static final int ELEVATOR_DRIVE_SHIFT_B = 3;
    public static final boolean RIGHT_SENSOR_PHASE = false; //Inverts Sensor
    public static final boolean LEFT_SENSOR_PHASE = false; //Inverts Sensor
    public static final double GEAR_RATIO = 23.54;
    public static final double WHEEL_RADIUS_INCHES = 4.0;
    public static final double TREAD_WIDTH_INCHES = 25;
    public static final double KS_DRIVE = 0.0;
    public static final double KV_DRIVE = 0.0;
    public static final double KA_DRIVE = 0.0;
    public static final double KP_DRIVE = 0.0;
    public static final double KI_DRIVE = 0.0;
    public static final double KD_DRIVE = 0.0;
    public static final double MAX_VELOCITY_FEET = 15.0;
    public static final double MAX_ACCELERATION_FEET = 15.0;
    public static final int TOLERABLE_SHIFT_VELOCITY_RPM = 500;
    public static final double CLOSED_LOOP_RAMP_RATE = 1;
    public static final double OPEN_LOOP_RAMP_RATE = 1; 
    //Intake Constants
    public static final int INTAKE_MOTOR = 0;
    public static final double INTAKE_FORWARD_SPEED = 0.5;
    public static final double INTAKE_REVERSE_SPEED = 0.5;
    public static final int INTAKE_SOLENOID_A = 4;
    public static final int INTAKE_SOLENOID_B = 5;
    //Turret Constants
    public static final int TURRET_MOTOR = 1;  
    public static final int HOOD_MOTOR = 2;
    public static final int HOOD_ENCODER_A = 2;
    public static final int HOOD_ENCODER_B = 2;
    public static final int HOOD_LIMIT = 0;
    public static final double HOOD_SPEED = 0.3;
}
