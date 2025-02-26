package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.MotionProfile.MotionConstraints;

public class Constants {

    public class DriveConstants {

        public static final boolean fieldRelative = true;

        public static final int frontRightDriveCANId = 19;
        public static final int frontLeftDriveCANId = 0;
        public static final int backRightDriveCANId = 10;
        public static final int backLeftDriveCANId = 9;

        public static final int frontRightTurnCANId = 18;
        public static final int backRightTurnCANId = 11;
        public static final int backLeftTurnCANId = 8;
        public static final int frontLeftTurnCANId = 1;

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
        public static final double maxAccelerationMetersPerSec2 = 5; // TODO: make this a real number

        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        public static final double kWheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d((kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((kWheelBase / 2) - .0508, -kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, -kTrackWidth / 2)
            );

        public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

        public static final double angleTolerance = 1;
        public static final double velocityTolerance = 0.1;
        public static final double positionTolerance = 0.05;
        public static final double xandyvelocityTolerance = 0.05;

        public static Transform3d cameraLocation = new Transform3d(0, 0, 0, new Rotation3d(0, Units.degreesToRadians(5), 0));
    }

    public enum CoralMechanismState {
        L1,
        L2,
        L3,
        L4,
        Stow,
        DeepCage,
        ShallowCage,
        CoralStation,
        Climb,
    }

    public enum AlgaeMechanismState {
        Out,
        Stow,
        Processor,
    }

    public class PivotConstants {

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A32%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A14%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A225%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static double velocityMaxError = 3;
        public static double positionMaxError = 1;
        public static double maxJerk = 3500;
        public static double maxAccel = 3200; // Max = 12,146 degrees/s^2
        public static double maxVelocity = 110; // Max = 151 degrees/s
        public static double velocityTolerance = 3;

        public static double kP = 0.03;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kV = 4.38; // V*s/rad
        public static double kG = 0.37;
        public static double kA = 0.02; // V*s^2/rad

        // Units = degrees. 0 degrees is pivot level (folded flat) to make FF work
        // right.
        public static double L1 = 0;
        public static double L2 = 85;
        public static double L3 = 85;
        public static double L4 = 90;
        public static double stow = 50;
        public static double deepCage = 90;
        public static double shallowCage = 0;
        public static double coralStation = 42;

        public static double forwardLimit = 0;
        public static double reverseLimit = 0;
    }

    public class OperatorConstants {

        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; // TODO: find good deadzone values for the xbox controllers

        public enum DriveType {
            JOYSTICK,
            XBOX,
        }
    }

    public class ElevatorConstants {

        // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A18%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A6%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A2.256%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A60%2C%22u%22%3A%22in%22%7D
        public static double maxElevatorJerk = 10000;
        public static double maxAccel = 240; // max @40A is 830 in/s^2
        public static double maxVelocity = 24; // max is ~121 in/s
        public static double velocityTolerance = 0.5;
        public static double velocityMaxError = 0.3;
        public static double positionMaxError = 0.4;
        public static double kP = 0.5;
        public static double kI;
        public static double kD;
        public static double kV = 0.13; // V*s/in
        public static double kA = 0.0013; // V*2^2/in
        public static double kS;
        public static double kG = .4;
        public static double statorCurrentLimit = 40;
        public static double conversionFactor = .825;//(9 / 54) * 2.256 * Math.PI;
        public static double dangerZone = 3.5; // elevator needs to be up at least this high for wrist to fold back

        public static double L1 = 0;
        public static double L2 = 0;
        public static double L3 = 12.3;
        public static double L4 = 35.5;
        public static double stow = 0;
        public static double deepCage = 4;
        public static double shallowCage = 0;
        public static double coralStation = 3.1;

        public static double forwardLimit = 64;
        public static double reverseLimit = 0;
    }

    public class WristConstants {

        //https://www.reca.lc/arm?armMass=%7B%22s%22%3A12.5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A30%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A70.4%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        //Actual gear ratio is different than expected due to the "coin paradox". 10 tooth gear rotatng around 44 tooth is 5.4 to 1 not 4.4.
        public static double wristCurrent = 30;
        public static double kP = .3; //last is .42
        public static double kI = 0;
        public static double kD = 0.003;
        public static double wrapPoint = .5;
        public static double sensorOffset = -0.6344921875;
        public static SensorDirectionValue sensorDirection =
            SensorDirectionValue.CounterClockwise_Positive;
        public static double kG = 0.1675;
        public static double kV = 1.1; // V/s*rad
        public static double kA = 0;
        public static double kS = 0.0575;

        public static double maxAccel = 3000;// last is 560 // max accel = 15k deg/s^2
        public static double maxJerk = 100000;
        public static double maxVelocity = 300;//last is 56 // max 560 deg/s
        public static double velocityTolerance = 3;

        public static double maxPositionTolerance = .8;
        public static double maxVelocityTolerance = 3;

        public static double dangerZone = 72; // Wrist cannot travel up past this angle unless elevator is up a bit.

        // Units = degrees. Wrist pointing with coral vertical is 0 degrees to make FF
        // correct
        public static double L1 = 0;
        public static double L2 = 60;
        public static double L3 = 60;
        public static double L4 = 50;
        public static double stow = 36;
        public static double deepCage = 115;
        public static double shallowCage = 0;
        public static double coralStation = 21;
    }

    public class CoralIntakeConstants {

        public static double currentLimit = 30;

        // LaserCAN region of interest values, please configure in grapple hook app i
        // think
        public static int ROIx = 0;
        public static int ROIy = 0;
        public static int ROIw = 0;
        public static int ROIh = 0;
        // units are in mm
        public static double sensorDistance = 5;
    }

    public class AlgaeIntakeConstants {
        public static double outPosition;
        public static double inPosition;
        public static double maxJerk = 100000;
        public static double maxAcceleration;
        public static double maxVelocity;
        public static double velocityTolerance;
        public static double velocityMaxError = 0.5; //degrees per second
        public static double positionMaxError = 2;//degrees
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kV = 0;
        public static double kG = 0;
        public static double positionConversionFactor = 0;
        public static double velocityConversionFactor = 0;
        public static double currentDetectionLimit = 25;//probably too low

        public static MotionProfile.MotionConstraints constraints = new MotionConstraints(maxJerk, maxAcceleration, maxVelocity, velocityTolerance);

    }
}
