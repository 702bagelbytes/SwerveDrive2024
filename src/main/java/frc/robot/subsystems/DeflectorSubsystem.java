package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeflectorSubsystem extends SubsystemBase {
    CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Constants.DeflectorConstants.STATOR_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(Constants.DeflectorConstants.ENABLE_STATOR_CURRENT_LIMIT)
    .withSupplyCurrentLimit(Constants.DeflectorConstants.CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(Constants.DeflectorConstants.ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(Constants.DeflectorConstants.CURRENT_THRESHOLD)
    .withSupplyTimeThreshold(Constants.DeflectorConstants.CURRENT_THRESHOLD_TIME);

    private TalonFX DeflectorMotor = new TalonFX(Constants.DeflectorConstants.DeflectorMotorID);

    public DeflectorSubsystem() {
        var limitConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(Constants.DeflectorConstants.DeflectorLimitEnable)
        .withForwardSoftLimitThreshold(DegToTick(Constants.DeflectorConstants.DeflectorPosStowValue))
        .withReverseSoftLimitThreshold(DegToTick(Constants.DeflectorConstants.DeflectorPosInValue))
        .withReverseSoftLimitEnable(Constants.DeflectorConstants.DeflectorLimitEnable);

        DeflectorMotor.setNeutralMode(NeutralModeValue.Brake);
        DeflectorMotor.setInverted(false);
        DeflectorMotor.getConfigurator().apply(limitConfigs);
        DeflectorMotor.getConfigurator().apply(currentlimits);
    }

    public void ResetArmPos() {
        DeflectorMotor.setPosition(0);
    }

    public double TickToDeg(double tick) {
        return tick * 9/2;
    }

    public double DegToTick(double deg) {
        return deg * 2/9;
    }

    public double getArmAngle() {
        return TickToDeg(DeflectorMotor.getPosition().getValueAsDouble());
    }

    public void set(double value) {
        DeflectorMotor.set(value);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Deflector Angle", getArmAngle());
        SmartDashboard.putNumber("Deflector", DeflectorMotor.getPosition().getValueAsDouble());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));

    }
}
