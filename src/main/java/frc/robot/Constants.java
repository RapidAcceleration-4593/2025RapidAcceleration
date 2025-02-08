// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (90) * 0.453592; // 90 Pounds to Kilograms.
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag.
    public static final double MAX_SPEED = Units.feetToMeters(14.5); // Maximum speed of robot in meters per second, used to limit acceleration.

    public static final class ElevatorConstants {
        public static final PIDConstants ELEVATOR_PID = new PIDConstants(0.001, 0.00075, 0); // TODO: Tune PID.
        
        public static final double MANUAL_CONTROL_SPEED = 0.5; // Speed (0 to 1).
        public static final int PID_THRESHOLD = 10;

        public static final SparkMax leftElevatorMotor = new SparkMax(1, MotorType.kBrushless);
        public static final SparkMax rightElevatorMotor = new SparkMax(4, MotorType.kBrushless);

        public static final Encoder heightEncoder = new Encoder(8, 9);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(7);
        public static final DigitalInput topLimitSwitch = new DigitalInput(6);

        public enum ElevatorStates {
            BOTTOM,
            PICKUP,
            L3,
            L4
        }
    }

    public static final class ArmConstants {
        public static final PIDConstants ARM_PID = new PIDConstants(0, 0, 0); // TODO: Tune PID.

        public static final double MANUAL_CONTROL_SPEED = 1.0; // Speed (0 to 1).
        public static final int PLACE_ROTATION_AMOUNT = 100; // Encoder ticks.

        public static final SparkMax armMotor = new SparkMax(7, MotorType.kBrushless);
        public static final Encoder armEncoder = new Encoder(0, 1);

        public static final DigitalInput topLimitSwitch = new DigitalInput(2);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(3);

        public static final int PID_THRESHOLD = 10;
        
        public enum ArmStates {
            BOTTOM,
            L1,
            L2,
            L3,
            L4
        }
    }

    public static final class IntakeConstants {
        public static final double MOTOR_STALL_AMPERAGE = 20; // In Amps.
        
        public static final double EXTENSION_MOTOR_SPEED = 0.5; // Speed (0 to 1).
        public static final double INTAKE_MOTOR_SPEED = 0.5; // Speed (0 to 1).

        public static final SparkMax leftExtensionMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.
        public static final SparkMax rightExtensionMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.

        public static final SparkMax leftIntakeMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.
        public static final SparkMax rightIntakeMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.

        // TODO: Remove Limit Switches.
        public static final DigitalInput leftLimitSwitch = new DigitalInput(0); // TODO: Assign Limit Switch Channel.
        public static final DigitalInput rightLimitSwitch = new DigitalInput(0); // TODO: Assign Limit Switch Channel.
    
        public enum IntakeStates {
            EXTENDED,
            RETRACTED
        }
    }

    public static final class SerializerConstants {
        public static final SparkMax beltMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.
        public static final DigitalInput beltLimitSwitch = new DigitalInput(0); // TODO: Assign Limit Switch Channel.
    }

    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);

        public static final double DISTANCE_FROM_REEF = Units.inchesToMeters(16.25 + 25);
        public static final boolean DRIVE_WITH_VISION = false;
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.875);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final double[] BLUE_REEF_POSE = {4.4895, 4.0259};
        public static final double[] RED_REEF_POSE = {13.0588, 4.0259};
    }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled, in seconds
        public static final double WHEEL_LOCK_TIME = 10;
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 0.9;
    }
}
