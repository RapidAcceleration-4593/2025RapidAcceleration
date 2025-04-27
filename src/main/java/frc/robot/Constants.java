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

        public static final SparkMax leftElevatorMotor = new SparkMax(1, MotorType.kBrushless);
        public static final SparkMax rightElevatorMotor = new SparkMax(6, MotorType.kBrushless);

        public static final Encoder elevatorEncoder = new Encoder(8, 9);

        public static final DigitalInput bottomLimitSwitch = new DigitalInput(7);
        public static final DigitalInput topLimitSwitch = new DigitalInput(6);
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

        public static final SparkMax armMotor = new SparkMax(8, MotorType.kBrushless);
        public static final Encoder armEncoder = new Encoder(0, 1);

        public static final DigitalInput topLimitSwitch = new DigitalInput(2);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(3);
    }

    public static final class IntakeConstants {
        public static final class IntakePIDConstants {
            public static final PIDConstants INTAKE_PID = new PIDConstants(0.001, 0.0, 0.0);
            public static final int TOLERANCE = 10;

            public static final double MAX_VELOCITY = 1000;
            public static final double MAX_ACCELERATION = 1500;
        }

        public static final SparkMax innerIntakeMotor = new SparkMax(7, MotorType.kBrushless);
        public static final SparkMax outerIntakeMotor = new SparkMax(2, MotorType.kBrushless);

        public static final SparkMax leaderDeployMotor = new SparkMax(4, MotorType.kBrushless);
        public static final SparkMax followerDeployMotor = new SparkMax(5, MotorType.kBrushless);

        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = 0.25;
        public static final double DEPLOY_SPEED = 0.25;

        public static final int SPIKE_CURRENT = 10;
    }

    public static final class SerializerConstants {
        public static final SparkMax serializerMotor = new SparkMax(3, MotorType.kBrushless);
        public static final DigitalInput serializerSensor = new DigitalInput(4);

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

        public enum ElevatorDirections {
            UP, DOWN
        }

        public enum ArmStates {
            BOTTOM, L2, TOP
        }

        public enum ArmDirections {
            UP, DOWN
        }

        public enum StartingPosition {
            LEFT, CENTER, RIGHT
        }

        public enum IntakeStates {
            IN, L1, OUT
        }

        public enum IntakeDirections {
            UP, DOWN
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
