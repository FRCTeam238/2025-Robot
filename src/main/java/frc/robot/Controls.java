package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

public class Controls {

    static SendableChooser<DriveType> driveTypeChooser = new SendableChooser<DriveType>();

    public static CommandXboxController controller = new CommandXboxController(0);
    public static CommandXboxController driverController = new CommandXboxController(3);
    public static CommandJoystick rightJoystick = new CommandJoystick(1); 
    public static CommandJoystick leftJoystick = new CommandJoystick(2); 

    static DriveType driveType = DriveType.JOYSTICK;

    @Logged
    public static double[] getSwerveJoystickValues() {
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

    private static boolean getSlowmode() {
        return leftJoystick.getHID().getRawButton(3) || rightJoystick.getHID().getRawButton(3);
      }

    public static DriveType getDriveType() {
        driveType = driveTypeChooser.getSelected();
        return driveType;
      }


    public enum DriveType {
        XBOX,
        JOYSTICK
    }
}
