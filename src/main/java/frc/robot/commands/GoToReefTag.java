package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

@Logged
public class GoToReefTag extends Command {

    Drivetrain drivetrain;

    ChassisSpeeds speeds = new ChassisSpeeds();

    Pose2d nearestPose;

    boolean rightSide = false;

    PIDController xController = new PIDController(10, 0, .5);
    PIDController yController = new PIDController(10, 0, .5);
    PIDController thetaController = new PIDController(10, 0, .5);

    Transform3d delta = new Transform3d();

    public GoToReefTag(boolean rightSide) {
        this.rightSide = rightSide;
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(0.05, 0.8);
        yController.setTolerance(0.05, 0.8);
        thetaController.setTolerance(.02, 0.8);
    }

    @Override
    public void initialize() {
        nearestPose = drivetrain.getPose().nearest(drivetrain.getReefPoses(rightSide));
        xController.setSetpoint(nearestPose.getX());
        thetaController.setSetpoint(nearestPose.getRotation().getDegrees());
        yController.setSetpoint(nearestPose.getY());
    }

    @Override
    public void execute() {

        var xSpeed = xController.calculate(drivetrain.getPose().getX());
        var ySpeed = yController.calculate(drivetrain.getPose().getY());
        var thetaSpeed = thetaController.calculate(drivetrain.getPose().getRotation().getRadians());

        drivetrain.driveFieldRelative(xSpeed, ySpeed, thetaSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveFieldRelative(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

}
