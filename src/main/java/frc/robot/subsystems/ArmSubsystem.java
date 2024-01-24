package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX ArmMotor = new TalonFX(Constants.ArmConstants.ArmMotorID);

    public ArmSubsystem() {
        ArmMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void ResetArmPos() {
        ArmMotor.setPosition(0);
    }

    public double TickToDeg(double tick) {
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
