package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class DriveConstants {
        public static final boolean fieldRelative = true;

        public static final int frontRightDriveCANId = 18;
        public static final int frontLeftDriveCANId = 11;
        public static final int backRightDriveCANId = 17;
        public static final int backLeftDriveCANId = 13;

        public static final int frontRightTurnCANId = 19;
        public static final int backRightTurnCANId = 17;
        public static final int backLeftTurnCANId = 12;
        public static final int frontLeftTurnCANId = 10;

        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0.01;

        public static final double kPAngular = 1;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0;

        public static final double llkP = .2;
        public static final double llkI = 0;
        public static final double llkD = 0;

        public static final double maxVelocityMetersPerSec = 4.3;
        public static final double maxAccelerationMetersPerSec2 = 100; // TODO: make this a real number

        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        public static final double kWheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d((kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((kWheelBase / 2) - .0508, -kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, -kTrackWidth / 2));

        public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

        public static final double turnTolerance = 0;
        public static final double velocityTolerance = 0.1;
        public static final double positionTolerance = 0.05;
        public static final double xandyvelocityTolerance = 0.05;

    }

    public class OperatorConstants {
        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; // TODO: find good deadzone values for the xbox controllers

        public enum DriveType {
            JOYSTICK,
            XBOX
        }
    }

    public class ElevatorConstants {
        public static double maxElevatorJerk;
        public static double maxAccel;
        public static double maxVelocity;
        public static double velocityTolerance;
    }
}
