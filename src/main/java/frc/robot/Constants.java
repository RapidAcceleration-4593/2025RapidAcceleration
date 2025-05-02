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
    public static final double ROBOT_MASS = Units.lbsToKilograms(130);
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag.
    public static final double MAX_SPEED = Units.feetToMeters(10.0); // Maximum speed of robot in meters per second, used to limit acceleration.

    public static final class ElevatorConstants {
        public static final class ElevatorPIDConstants {
            public static final PIDConstants ELEVATOR_PID = new PIDConstants(0.000475, 0.00002, 0);
            public static final int TOLERANCE = 100;

            public static final double MAX_VELOCITY = 23000;
            public static final double MAX_ACCELERATION = 120000;
        }

        public static final class ElevatorTravelTime {
            public static final double MAX_TRAVEL = 1.5;
            public static final double KAH_CHUNK = 0.4;
        }

        public static final double CONTROL_SPEED = 0.5;

        public static final SparkMax LEFT_ELEVATOR_MOTOR = new SparkMax(1, MotorType.kBrushless);
        public static final SparkMax RIGHT_ELEVATOR_MOTOR = new SparkMax(6, MotorType.kBrushless);

        public static final Encoder ELEVATOR_ENCODER = new Encoder(8, 9);

        public static final DigitalInput BOTTOM_LIMIT_SWITCH = new DigitalInput(7);
        public static final DigitalInput TOP_LIMIT_SWITCH = new DigitalInput(6);
    }

    public static final class ArmConstants {
        public static final class ArmPIDConstants {
            public static final PIDConstants ARM_PID = new PIDConstants(0.008, 0.0004, 0);
            public static final int TOLERANCE = 20;

            public static final double MAX_VELOCITY = 1240;
            public static final double MAX_ACCELERATION = 5000;
        }

        public static final class ArmTravelTime {
            public static final double MAX_TRAVEL = 1.4;
            public static final double SCORE = 0.4;
        }

        public static final int PLACE_ROTATION_AMOUNT = 225;
        public static final double CONTROL_SPEED = 0.5;

        public static final SparkMax ARM_MOTOR = new SparkMax(8, MotorType.kBrushless);
        public static final Encoder ARM_ENCODER = new Encoder(0, 1);

        public static final DigitalInput TOP_LIMIT_SWITCH = new DigitalInput(2);
        public static final DigitalInput BOTTOM_LIMIT_SWITCH = new DigitalInput(3);
    }

    public static final class IntakeConstants {
        public static final class IntakePIDConstants {
            public static final PIDConstants INTAKE_PID = new PIDConstants(0.0003, 0.0, 0.0);
            public static final int TOLERANCE = 100;

            // public static final double MAX_VELOCITY = 1000; // 3400
            // public static final double MAX_ACCELERATION = 800; // 800
        }

        public static final SparkMax LEFT_INTAKE_MOTOR = new SparkMax(2, MotorType.kBrushless);
        public static final SparkMax RIGHT_INTAKE_MOTOR = new SparkMax(7, MotorType.kBrushless);

        public static final SparkMax LEFT_DEPLOY_MOTOR = new SparkMax(4, MotorType.kBrushless);
        public static final SparkMax RIGHT_DEPLOY_MOTOR = new SparkMax(5, MotorType.kBrushless);

        public static final double INTAKE_SPEED = 0.8;
        public static final double OUTTAKE_SPEED = 0.3;
        public static final double DEPLOY_SPEED = 0.75;
    }

    public static final class SerializerConstants {
        public static final SparkMax SERIALIZER_MOTOR = new SparkMax(3, MotorType.kBrushless);
        public static final DigitalInput SERIALIZER_DISTANCE_SENSOR = new DigitalInput(4);

        public static final double CONTROL_SPEED = 0.4;
        public static final double MAX_TIMEOUT = 1.75;
    }

    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);

        public static final boolean DRIVE_WITH_VISION = true;

        public static final double MAX_VELOCITY = 2.0;
        public static final double MAX_ACCELERATION = 1.75;

        public static final class DashboardAlignment {
            public static final double DISTANCE_AT_REEF = Units.inchesToMeters(17);
            public static final double DISTANCE_AWAY_REEF = Units.inchesToMeters(50);
            public static final double BRANCH_OFFSET = Units.inchesToMeters(6.5);
        }
    }

    public static final class RobotStates {
        public enum ElevatorStates {
            BOTTOM, PICKUP, TOP
        }

        public enum ArmStates {
            BOTTOM, L2, TOP
        }

        public enum IntakeStates {
            IN, L1, OUT
        }

        public enum ControlDirections {
            UP, DOWN
        }

        public enum StartingPosition {
            LEFT, CENTER, RIGHT
        }
    }

    public static final class DrivebaseConstants {
        public static final double WHEEL_LOCK_TIME = 10; // Seconds.
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUXILIARY_CONTROLLER_PORT = 1;

        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 1.0;
    }
}
