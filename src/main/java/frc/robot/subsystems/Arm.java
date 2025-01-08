package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.kD;
import static frc.robot.Constants.DriveConstants.kI;
import static frc.robot.Constants.DriveConstants.kP;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    TalonFX motor;

    public Arm() {
        motor = new TalonFX(Constants.ArmConstants.motorID);
        var config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }

}
