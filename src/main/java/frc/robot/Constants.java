// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (110) * 0.453592; // 115 Pounds to Kilograms.
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag.
    public static final double MAX_SPEED = Units.feetToMeters(8.0); // Maximum speed of robot in meters per second, used to limit acceleration.

    public static final class ElevatorConstants {
        public static final class ElevatorPIDConstants {
            public static final PIDConstants ELEVATOR_PID = new PIDConstants(0.00058, 0, 0); // TODO: Tune PID.
            public static final int TOLERANCE = 30;

            public static final double MAX_VELOCITY = 23000; // Previously: 23000
            public static final double MAX_ACCELERATION = 120000; // Previously: 120000
        }

        public static final double CONTROL_SPEED = 0.6;

        public static final SparkMax leftElevatorMotor = new SparkMax(1, MotorType.kBrushless);
        public static final SparkMax rightElevatorMotor = new SparkMax(6, MotorType.kBrushless);

        public static final Encoder elevatorEncoder = new Encoder(8, 9);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(7);
        public static final DigitalInput topLimitSwitch = new DigitalInput(6);
    }

    public static final class ArmConstants {
        public static final class ArmPIDConstants {
            public static final PIDConstants ARM_PID = new PIDConstants(0.01, 0.03, 0); // TODO: Tune PID.
            public static final int TOLERANCE = 15;

            public static final double MAX_VELOCITY = 1300; // Previously: 1600
            public static final double MAX_ACCELERATION = 5000; // Previously: 9500
        }

        public static final int PLACE_ROTATION_AMOUNT = 180;
        public static final double CONTROL_SPEED = 0.8;

        public static final SparkMax armMotor = new SparkMax(7, MotorType.kBrushless);
        public static final Encoder armEncoder = new Encoder(0, 1);

        public static final DigitalInput topLimitSwitch = new DigitalInput(2);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(3);
    }

    public static final class SerializerConstants {
        public static final SparkMax serializerMotor = new SparkMax(8, MotorType.kBrushless);

        public static final double CONTROL_SPEED = 0.35;
    }

    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);

        public static final boolean DRIVE_WITH_VISION = true;

        public static final class DashboardAlignment {
            public static final double DISTANCE_FROM_REEF = Units.inchesToMeters(19.0);
            public static final double REEF_RADIUS = Units.inchesToMeters(32.75);
            public static final double BRANCH_OFFSET = Units.inchesToMeters(6.25);
            public static final double ANGLE_INCREMENT = Math.toRadians(60.0);

        }
    }

    public static final class RobotStates {
        public static final class Elevator {
            public enum ElevatorStates {
                BOTTOM, PICKUP, TOP
            }

            public enum ElevatorDirections {
                UP, DOWN
            }
        }

        public static final class Arm {
            public enum ArmStates {
                BOTTOM, L2, TOP
            }
    
            public enum ArmDirections {
                UP, DOWN, UNKNOWN
            }
        }

        public static final class Autonomous {
            public enum StartingPosition {
                LEFT, CENTER, RIGHT
            }
        }
    }

    public static final class FieldConstants {
        public static final Pose2d REEF_POSE = new Pose2d(new Translation2d(4.4895, 4.0259), new Rotation2d(0));
    }

    public static final class DrivebaseConstants {
        public static final double WHEEL_LOCK_TIME = 10; // Seconds.
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUXILIARY_CONTROLLER_PORT = 1;

        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 0.8;
    }
}
