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
public class GoToReefTag extends Command{

    Drivetrain drivetrain;

    ChassisSpeeds speeds = new ChassisSpeeds();

    Pose2d nearestPose;
    
    boolean rightSide = false;
    double desiredX = 0.2; //X in cam coordinates, increases as distance is further away, Y in robot coordinates
    double desiredY = -.03; //Y in cam coordinates, + to the left, negative to the right from cam, X in robot coordinates
    double desiredTheta = -170; //Clockwise positive, reported angle is -180 to 180, make sure to handle wrap

    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    PIDController thetaController = new PIDController(0,0,0);

    Transform3d delta = new Transform3d();
    
    public GoToReefTag(boolean rightSide) {
        this.rightSide = rightSide;
        drivetrain = Drivetrain.getInstance();
        //addRequirements(drivetrain);
        thetaController.enableContinuousInput(-180, 180);
        // xController.setSetpoint(desiredX);
        xController.setTolerance(0.05, 0.8);
        yController.setTolerance(0.05, 0.8);
        thetaController.setTolerance(1, 0.8);
        // yController.setSetpoint(desiredY);
        // thetaController.setSetpoint(desiredTheta);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        //tagID = drivetrain.getBestTagId(rightSide);
        nearestPose = drivetrain.getPose().nearest(drivetrain.getReefPoses(rightSide));
        xController.setSetpoint(nearestPose.getX());
        thetaController.setSetpoint(nearestPose.getRotation().getDegrees());
        yController.setSetpoint(nearestPose.getY());
    }

    @Override
    public void execute() {
        
        var xSpeed = xController.calculate(drivetrain.getPose().getX());
        var ySpeed = yController.calculate(drivetrain.getPose().getY());
        var thetaSpeed = thetaController.calculate(drivetrain.getPose().getRotation().getDegrees());

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
