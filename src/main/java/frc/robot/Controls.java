package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

@Logged
public class Controls {

    private static Controls singleton;

    static SendableChooser<DriveType> driveTypeChooser = new SendableChooser<DriveType>();

    @NotLogged CommandXboxController controller = new CommandXboxController(0);
    @NotLogged CommandXboxController driverController = new CommandXboxController(3);
    @NotLogged CommandJoystick rightJoystick = new CommandJoystick(1); 
    @NotLogged CommandJoystick leftJoystick = new CommandJoystick(2); 

    DriveType driveType = DriveType.JOYSTICK;

    private Controls(){
        driveTypeChooser.addOption("XBOX", DriveType.XBOX);
        driveTypeChooser.setDefaultOption("JOYSTICK", DriveType.JOYSTICK);
        SmartDashboard.putData(driveTypeChooser);
    }

    public static Controls getInstance()
    {
        if (singleton == null)
        {
            singleton = new Controls();
        }
        return singleton;
    }

    @Logged
    public double[] getSwerveJoystickValues() {
        double slowmodePercent = getSlowmode() ? .75 : 1;

        switch (getDriveType()) {
            case JOYSTICK -> {
                return new double[] {
                        // applyDeadband will do the absolute value stuff for us and make the zero point
                        // start at
                        // the deadzone edge
                        Math.pow(-MathUtil.applyDeadband(leftJoystick.getY(), driverJoystickDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(leftJoystick.getX(), driverJoystickDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(rightJoystick.getX(), driverJoystickDeadzone), 1),
                };
            }
            case XBOX -> {
                return new double[] {
                        // applyDeadband will do the absolute value stuff for us and make the zero point
                        // start at
                        // the deadzone edge
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftY(), xboxControllerDeadzone), 5)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftX(), xboxControllerDeadzone), 5)
                                * slowmodePercent,
                        Math.pow(
                                -MathUtil.applyDeadband(driverController.getRightX(), xboxControllerDeadzone), 3),
                };
            }
            default -> throw new IllegalStateException("Unexpected value: " + getDriveType());
        }
    }

    private boolean getSlowmode() {
        return leftJoystick.getHID().getRawButton(3) || rightJoystick.getHID().getRawButton(3);
      }

    public DriveType getDriveType() {
        driveType = driveTypeChooser.getSelected();
        return driveType;
      }


    public enum DriveType {
        XBOX,
        JOYSTICK
    }
}
