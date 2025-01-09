package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** for swerve */
@Logged
public class Drivetrain extends SubsystemBase {

  @NotLogged SwerveModule frontLeft = new SwerveModule(frontLeftDriveCANId, frontLeftTurnCANId);
  @NotLogged SwerveModule frontRight = new SwerveModule(frontRightDriveCANId, frontRightTurnCANId);
  @NotLogged SwerveModule backLeft = new SwerveModule(backLeftDriveCANId, backLeftTurnCANId);
  @NotLogged SwerveModule backRight = new SwerveModule(backRightDriveCANId, backRightTurnCANId);

  @NotLogged SwerveDrivePoseEstimator odometry;
  AHRS gyro;
  PIDController x, y, theta;
  String command = "None";
  SwerveSample trajectoryPose = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0});

  public Drivetrain() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    odometry =
        new SwerveDrivePoseEstimator(
            kDriveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            },
            new Pose2d());

    x = new PIDController(kP, kI, kD);
    y = new PIDController(kP, kI, kD);
    theta = new PIDController(kPAngular, kIAngular, kDAngular);
    theta.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setCommand(String name) {
    command = name;
  }

  // public void updatePoseEstimate(EstimatedRobotPose estimate) {
  //   odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
  // }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getFieldRelativeOffset() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return odometry.getEstimatedPosition().getRotation();
      } else {
        return odometry
            .getEstimatedPosition()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180)); // DO NOT USE IF WE DONT RUN A PATH
      }
    } else {
      return odometry.getEstimatedPosition().getRotation();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getFieldRelativeOffset())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public void lockWheels() {
    SwerveModuleState wheelLock[] = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    setModuleStates(wheelLock);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSec);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
    };
  }

  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
            frontLeft.getDesiredState(), frontRight.getDesiredState(), backLeft.getDesiredState(), backRight.getDesiredState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };
  }

  public void zeroHeading() {
    gyro.reset();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        gyro.setAngleAdjustment(180);
      } else {
        gyro.setAngleAdjustment(0);
      }
    }
  }

  public Command zeroHeadingCommand() {
    return Commands.runOnce(
        () -> {
          this.zeroHeading();
        });
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public ChassisSpeeds choreoController(SwerveSample referenceState) {
    Pose2d currentPose = getPose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = x.calculate(currentPose.getX(), referenceState.x);
    double yFeedback = y.calculate(currentPose.getY(), referenceState.y);
    double rotationFeedback =
        theta.calculate(currentPose.getRotation().getRadians(), referenceState.heading);
    
    ChassisSpeeds retval = ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    return retval;
  }
}
