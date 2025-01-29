// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auton.utils.AutonUtils;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (60) * 0.453592; // 60 lbs
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5); // Maximum speed of robot in meters per second, used to limit acceleration

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

        // TODO: Move to different class.
        public static final Pose2d[] BLUE_BOTTOM_CHUTE = {
            new Pose2d(0.5781, 1.3135, Rotation2d.fromDegrees(-126)),
            new Pose2d(1.0714, 0.9553, Rotation2d.fromDegrees(-126)),
            new Pose2d(1.5646, 0.5971, Rotation2d.fromDegrees(-126))
        };

        public static final Pose2d[] BLUE_TOP_CHUTE = {
            new Pose2d(0.5781, 6.7383, Rotation2d.fromDegrees(126)),
            new Pose2d(1.0714, 7.0965, Rotation2d.fromDegrees(126)),
            new Pose2d(1.5646, 7.4547, Rotation2d.fromDegrees(126))
        };

        public static final Pose2d[] RED_BOTTOM_CHUTE = flipFieldPoses(BLUE_BOTTOM_CHUTE);
        public static final Pose2d[] RED_TOP_CHUTE = flipFieldPoses(BLUE_BOTTOM_CHUTE);

        private static Pose2d[] flipFieldPoses(Pose2d[] bluePoses) {
            Pose2d[] redPoses = new Pose2d[bluePoses.length];
            for (int i = 0; i < bluePoses.length; i++) {
                redPoses[i] = AutonUtils.flipFieldPose(bluePoses[i]);
            }
            return redPoses;
        }
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
