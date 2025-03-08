package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeMechanismState;
import frc.robot.Constants.CoralMechanismState;
import frc.robot.commands.AlgaeProfile;
import frc.robot.commands.Drive;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.GoToReefTag;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.MechanismPosition;
import frc.robot.commands.RunAlgaeIntake;
import frc.robot.commands.SnapToAngle;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.OperatorConstants.*;

@Logged
public class Controls {

    private static Controls singleton;

    static SendableChooser<DriveType> driveTypeChooser = new SendableChooser<DriveType>();

    @NotLogged
    CommandXboxController controller = new CommandXboxController(0);
    @NotLogged
    CommandXboxController driverController = new CommandXboxController(3);
    @NotLogged
    CommandJoystick rightJoystick = new CommandJoystick(1);
    @NotLogged
    CommandJoystick leftJoystick = new CommandJoystick(2);

    DriveType driveType = DriveType.JOYSTICK;

    GoToReefTag reefCommand = new GoToReefTag(false);

    private Controls() {
        CommandScheduler.getInstance().schedule(reefCommand);
        DriverStation.silenceJoystickConnectionWarning(true);
        driveTypeChooser.addOption("JOYSTICK", DriveType.XBOX);
        driveTypeChooser.setDefaultOption("XBOX", DriveType.JOYSTICK);
        SmartDashboard.putData(driveTypeChooser);

        driverController.start().onTrue(Drivetrain.getInstance().zeroHeadingCommand());
        leftJoystick.button(4).onTrue(Drivetrain.getInstance().zeroHeadingCommand());
        rightJoystick.button(4).onTrue(Drivetrain.getInstance().zeroHeadingCommand());

        Drivetrain.getInstance().setDefaultCommand(new Drive());
        // Pivot.getInstance().setDefaultCommand(Pivot.getInstance().holdPositionCommand());
        // Elevator.getInstance().setDefaultCommand(new ManualElevator());
        // AlgaeIntake.getInstance().setDefaultCommand(new AlgaeProfile(AlgaeMechanismState.Stow));
        // controller.a().onTrue(new MechanismPosition(CoralMechanismState.L1));
        controller.x().onTrue(new MechanismPosition(CoralMechanismState.L2));
        controller.b().onTrue(new MechanismPosition(CoralMechanismState.L3));
        controller.y().onTrue(new MechanismPosition(CoralMechanismState.L4));
        controller.rightBumper().onTrue(new MechanismPosition(CoralMechanismState.CoralStation));
        controller.povDown().onTrue(new MechanismPosition(CoralMechanismState.Stow));
        controller.povUp().onTrue(new MechanismPosition(CoralMechanismState.DeepCage));

        // controller.leftTrigger().whileTrue(new AlgaeProfile(AlgaeMechanismState.Out).andThen(new RunAlgaeIntake(false)).andThen(rumbleCommand())).onFalse(new AlgaeProfile(AlgaeMechanismState.Stow));
        // controller.leftBumper().whileTrue(new AlgaeProfile(AlgaeMechanismState.Out).andThen(new RunAlgaeIntake(false)).andThen(rumbleCommand())).onFalse(new AlgaeProfile(AlgaeMechanismState.Stow));
        controller.leftTrigger().whileTrue(new RunAlgaeIntake(false));
        controller.leftBumper().whileTrue(new AlgaeProfile(AlgaeMechanismState.Out)).onFalse(new AlgaeProfile(AlgaeMechanismState.Stow));

        leftJoystick.button(1).whileTrue(new RunAlgaeIntake(true));

        controller.rightTrigger().whileTrue(new IntakeCoral(false).andThen(new IntakeCoral(true).withTimeout(.1)).andThen(rumbleCommand()));

        controller.axisGreaterThan(1, 0.1).whileTrue(new ManualElevator()); // Left Y
        controller.axisLessThan(1, -0.1).whileTrue(new ManualElevator()); // Left Y
        controller.axisGreaterThan(5, 0.1).whileTrue(new ManualPivot()); // Right Y
        controller.axisLessThan(5, -0.1).whileTrue(new ManualPivot()); // Right Y
    

        leftJoystick.button(11).whileTrue(new SnapToAngle(0));
        leftJoystick.button(12).whileTrue(new SnapToAngle(90));
        leftJoystick.button(13).whileTrue(new SnapToAngle(180));
        leftJoystick.button(14).whileTrue(new SnapToAngle(270));

        leftJoystick.povUp().whileTrue(new SnapToAngle(60));
        leftJoystick.povRight().whileTrue(new SnapToAngle(120));
        leftJoystick.povLeft().whileTrue(new SnapToAngle(240));
        leftJoystick.povDown().whileTrue(new SnapToAngle(300));

        // leftJoystick.button(1).whileTrue(new EjectCoral());

        rightJoystick.button(11).whileTrue(new SnapToAngle(0));
        rightJoystick.button(12).whileTrue(new SnapToAngle(90));
        rightJoystick.button(13).whileTrue(new SnapToAngle(180));
        rightJoystick.button(14).whileTrue(new SnapToAngle(270));

        rightJoystick.povUp().whileTrue(new SnapToAngle(60));
        rightJoystick.povRight().whileTrue(new SnapToAngle(120));
        rightJoystick.povLeft().whileTrue(new SnapToAngle(240));
        rightJoystick.povDown().whileTrue(new SnapToAngle(300));

        rightJoystick.button(1).whileTrue(new EjectCoral());
    }

    public static Controls getInstance() {
        if (singleton == null) {
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
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftY(), xboxControllerDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(-MathUtil.applyDeadband(driverController.getLeftX(), xboxControllerDeadzone), 1)
                                * slowmodePercent,
                        Math.pow(
                                -MathUtil.applyDeadband(driverController.getRightX(), xboxControllerDeadzone), 1),
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

    public double getOperatorLeftStickY() {
        double x = controller.getLeftY();
        if (Math.abs(x) < 0.1) {
            x = 0;
        }

        return x;
    }

    public double getOperatorRightStickY() {
        double y = controller.getRightY();
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        return y;
    }

    public enum DriveType {
        XBOX,
        JOYSTICK
    }

    public Command rumbleCommand() {
        return new RunCommand(() -> {
            controller.setRumble(RumbleType.kBothRumble, 1);
        }).finallyDo(() -> {
            controller.setRumble(RumbleType.kBothRumble, 0);
        });
    }
}
