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
    public static final double ROBOT_MASS = (60) * 0.453592; // 60 lbs
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5); // Maximum speed of robot in meters per second, used to limit acceleration

    public static final class ElevatorConstants {
        public static final PIDConstants ELEVATOR_PID = new PIDConstants(0, 0, 0); // TODO: Tune PID.
        public static final double MANUAL_CONTROL_SPEED = 3000;
        public static final double PID_THRESHOLD = 0.005;

        public static final SparkMax leftElevatorMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.
        public static final SparkMax rightElevatorMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Assign Motor ID.

        public static final Encoder heightEncoder = new Encoder(0, 0); // TODO: Assign Encoder Channels.
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(0); // TODO: Assign Limit Switch Channel.
        public static final DigitalInput topLimitSwitch = new DigitalInput(0); // TODO: Assign Limit Switch Channel.

        public enum ElevatorLevel {
            BOTTOM,
            PICKUP,
            L3,
            L4
        }
    }

    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);

        public static final double DISTANCE_FROM_REEF = Units.inchesToMeters(16.25 + 25);
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
        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 0.75;
    }
}
