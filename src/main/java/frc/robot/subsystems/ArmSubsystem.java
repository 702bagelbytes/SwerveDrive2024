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

public class ArmSubsystem extends SubsystemBase {

    
    private TalonFX ArmMotor = new TalonFX(Constants.ArmConstants.ArmMotorID);

    public ArmSubsystem() {
        CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Constants.ArmConstants.STATOR_CURRENT_LIMIT)
    .withStatorCurrentLimitEnable(Constants.ArmConstants.ENABLE_STATOR_CURRENT_LIMIT)
    .withSupplyCurrentLimit(Constants.ArmConstants.CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(Constants.ArmConstants.ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(Constants.ArmConstants.CURRENT_THRESHOLD)
    .withSupplyTimeThreshold(Constants.ArmConstants.CURRENT_THRESHOLD_TIME);
   
    

        var limitConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(Constants.ArmConstants.ArmLimitEnable)
        .withForwardSoftLimitThreshold(DegToTick(Constants.ArmConstants.ArmPosInValue))
        .withReverseSoftLimitThreshold(DegToTick(Constants.ArmConstants.ArmPosOutValue))
        .withReverseSoftLimitEnable(Constants.ArmConstants.ArmLimitEnable);

        ArmMotor.setNeutralMode(NeutralModeValue.Brake);
        ArmMotor.getConfigurator().apply(limitConfigs);
        ArmMotor.getConfigurator().apply(currentlimits);
    }

    public void ResetArmPos() {
        ArmMotor.setPosition(0);
    }

    public double TickToDeg(double tick) {
        return tick;
    }

    public double DegToTick(double tick) {
        return tick;
    }

    public double getArmAngle() {
        return TickToDeg(ArmMotor.getPosition().getValueAsDouble());
    }

    public void set(double value) {
        ArmMotor.set(value);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));

    }
}
