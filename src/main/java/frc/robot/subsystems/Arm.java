package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubsystemConstants;

public class Arm extends SubsystemBase {

    private CANSparkMax motor;
    private RelativeEncoder encoder;

    private ProfiledPIDController controller;
    private ArmFeedforward feedforward;

    private double setpoint_rad = 0.0;
    private boolean homing = false;

    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.01;
    private static final double kG = 0.04;
    private static final double kV = 1.36;

    public Arm() {

        motor = new CANSparkMax(ArmSubsystemConstants.ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        Timer.delay(0.2);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(ArmSubsystemConstants.INVERTED);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ArmSubsystemConstants.POS_CONV_FACTOR);
        encoder.setVelocityConversionFactor(ArmSubsystemConstants.VELOCITY_CONV_FACTOR);
        motor.setSmartCurrentLimit(ArmSubsystemConstants.CURRENT_LIMIT);

        motor.burnFlash();
        Timer.delay(0.2);

        controller = new ProfiledPIDController(kP, kI, kD,
                new Constraints(ArmSubsystemConstants.MAX_SPEED_RAD_PER_SEC,
                        ArmSubsystemConstants.MAX_ACCEL_RAD_PER_SEC));
        controller.setTolerance(Units.degreesToRadians(2.0));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        feedforward = new ArmFeedforward(kS, kG, kV);

    }

    public void setPercent(double speed) {
        motor.set(speed);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint_rad = setpoint;
        controller.setGoal(new State(setpoint_rad, 0.0));
    }

    public void setAngle(double angle) {
        encoder.setPosition(angle);
    }

    public void setHoming(boolean homing) {
        this.homing = homing;
    }

    public double getAngle() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getWrappedAngle() {
        return MathUtil.angleModulus(getAngle());
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getAngle());
    }

    public boolean getHoming() {
        return homing;
    }

    public boolean atGoal() {
        return controller.atGoal();
    }

    public void resetController() {
        controller.reset(new State(getWrappedAngle(), getVelocity()));
    }

    public void stop() {
        motor.set(0.0);
    }

    @Override
    public void periodic() {

        if (homing) {
            motor.set(0.05);
        } else {
            motor.setVoltage(controller.calculate(getWrappedAngle())
                    + feedforward.calculate(getWrappedAngle(), controller.getSetpoint().velocity));
        }

    }

}
