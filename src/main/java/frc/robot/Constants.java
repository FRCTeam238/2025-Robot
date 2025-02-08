package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

    public enum CoralMechanismState {
        L1,
        L2,
        L3,
        L4,
        Stow,
        DeepCage,
        ShallowCage,
        CoralStation
    }

    public enum AlgaeMechanismState {
        Out,
        Stow,
        Processor //do we need this?
    }

    public class PivotConstants {
        public static double velocityMaxError = 0;
        public static double positionMaxError = 0;
        public static double maxJerk = 0;
        public static double maxAccel = 0;
        public static double maxVelocity = 0;
        public static double velocityTolerance = 0;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kV = 0;
        public static double kG = 0;
        // positions are measured in degrees
        public static double L1 = 0;
        public static double L2 = 0;
        public static double L3 = 0;
        public static double L4 = 0;
        public static double stow = 0;
        public static double deepCage = 0;
        public static double shallowCage = 0;
        public static double coralStation = 0;

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
        public static double velocityMaxError;
        public static double positionMaxError;
        public static double kP;
        public static double kI;
        public static double kD;
        public static double kV;
        public static double kS;
        public static double kG;
        public static double statorCurrentLimit = 40;
        public static double conversionFactor = (9/54) * 2.256 * Math.PI;

        public static double L1 = 0;
        public static double L2 = 0;
        public static double L3 = 0;
        public static double L4 = 0;
        public static double stow = 0;
        public static double deepCage = 0;
        public static double shallowCage = 0;
        public static double coralStation = 0;
    }

    public class WristConstants {
        public static double wristCurrent = 30;
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double wrapPoint = .5;
        public static double sensorOffset = 0;
        public static SensorDirectionValue sensorDirection = SensorDirectionValue.Clockwise_Positive;
        public static double kG = 0;
        public static double kV = 0;
        public static double kA = 0;
        public static double kS = 0;

        public static double maxAccel = 0;
        public static double maxJerk = 0;
        public static double maxVelocity = 0;
        public static double velocityTolerance = 0;

        public static double maxPositionTolerance = 0;
        public static double maxVelocityTolerance = 0;
        
        public static double L1 = 0;
        public static double L2 = 0;
        public static double L3 = 0;
        public static double L4 = 0;
        public static double stow = 0;
        public static double deepCage = 0;
        public static double shallowCage = 0;
        public static double coralStation = 0;

        
    }
        


    public class CoralIntakeConstants {
        public static double currentLimit = 30;


        //LaserCAN region of interest values, please configure in grapple hook app i think
        public static int ROIx = 0;
        public static int ROIy = 0;
        public static int ROIw = 0;
        public static int ROIh = 0;
        // units are in mm
        public static double sensorDistance = 0;
        
    }}
