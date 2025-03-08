package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

@Logged
public class GoToReefTag extends Command{

    Drivetrain drivetrain;

    ChassisSpeeds speeds = new ChassisSpeeds();

    int tagID = 21;
    
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
        xController.setSetpoint(desiredX);
        yController.setSetpoint(desiredY);
        thetaController.setSetpoint(desiredTheta);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        //tagID = drivetrain.getBestTagId(rightSide);
    }

    @Override
    public void execute() {
        //y: 7, x: 1, 0: 10
        for (var target : drivetrain.getLatestResult(rightSide).targets) {
            if (target.fiducialId == tagID) {
                xController.calculate(target.bestCameraToTarget.getX());
                yController.calculate(target.bestCameraToTarget.getY());
                thetaController.calculate(Units.radiansToDegrees(target.bestCameraToTarget.getRotation().getAngle()));
            }
        }
        
        // ChassisSpeeds speeds =

        // drivetrain.driveWithChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private boolean isReefTag(int tag, boolean isBlue) {
        return (tag <= 11 && tag >= 6 && !isBlue) || (tag <=22 && tag >= 17 && isBlue);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
