package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import static frc.robot.Constants.WristConstants.*;

@Logged
public class Wrist extends SubsystemBase {
    //talonfx - falcon
    TalonFX wristMotor = new TalonFX(0);
    CANcoder wristSensor = new CANcoder(0);
    ArmFeedforward wristFf = new ArmFeedforward(kS, kG, kV, kA);
    PositionVoltage wristVoltage = new PositionVoltage(0);
    String commandName = "";
    MotionProfile.State desiredState = new MotionProfile.State(0);

    private static Wrist singleton;

    private Wrist() {
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.CurrentLimits.StatorCurrentLimit = wristCurrent;
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        wristConfig.Feedback.FeedbackRemoteSensorID = wristSensor.getDeviceID();
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        wristConfig.Feedback.SensorToMechanismRatio = 1/360; //Native unit is rotations, this converts to degrees
        wristConfig.Slot0.kP = kP;
        wristConfig.Slot0.kI = kI;
        wristConfig.Slot0.kD = kD;
        wristMotor.getConfigurator().apply(wristConfig);

        CANcoderConfiguration sensorConfig = new CANcoderConfiguration();
        sensorConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = wrapPoint;
        sensorConfig.MagnetSensor.MagnetOffset = sensorOffset;
        sensorConfig.MagnetSensor.SensorDirection = sensorDirection;
        wristSensor.getConfigurator().apply(sensorConfig);
    }

    public static Wrist getInstance() {
        if (singleton == null) {
          singleton = new Wrist();
        }
        return singleton;
      }

    public void setSpeed(double speed) {
        wristMotor.set(speed);
    }
     
    public void setDesiredState(MotionProfile.State state) {
    desiredState = state;
    double wristAngle = state.position + 90 - Pivot.getInstance().getPosition(); 
    double feed = wristFf.calculate(Units.degreesToRadians(wristAngle), Units.degreesToRadians(state.velocity), Units.degreesToRadians(state.acceleration));
    
    wristVoltage.withFeedForward(feed).withPosition(state.position);
    wristMotor.setControl(wristVoltage);
    }

    public double getPosition() {
        return wristSensor.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public double getVelocity() {
        return wristSensor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        wristMotor.set(0);
    }

    public void setCommand(String name) {
        commandName = name;
    }

    public void holdPosition() {
        double wristAngle = getPosition() + Pivot.getInstance().getPosition(); 
        //Change 0 to angle from pivot
        double feed = wristFf.calculate(wristAngle, 0, 0);
        wristVoltage.withFeedForward(feed).Position = getPosition();
        wristMotor.setControl(wristVoltage);
    }
}
