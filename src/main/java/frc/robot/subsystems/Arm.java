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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.commands.ArmHoming;
import frc.robot.commands.SetGoal;

public class Arm extends SubsystemBase {

    private CANSparkMax motor;
    private RelativeEncoder encoder;

    private ProfiledPIDController controller;
    private ArmFeedforward feedforward;

    private double setpoint_rad = ArmSubsystemConstants.STRAIGHT_UP_STATE_RAD;
    private boolean homing = false;

    private static final double kP = 0.61;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.09;
    private static final double kG = 0.02;
    private static final double kV = 1.95;


    public Arm() {

        motor = new CANSparkMax(ArmSubsystemConstants.ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        Timer.delay(0.2);

        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(ArmSubsystemConstants.INVERTED);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ArmSubsystemConstants.POS_CONV_FACTOR);
        encoder.setVelocityConversionFactor(ArmSubsystemConstants.VELOCITY_CONV_FACTOR);
        motor.setSmartCurrentLimit(ArmSubsystemConstants.CURRENT_LIMIT);

        Timer.delay(0.2);
        motor.burnFlash();

        controller = new ProfiledPIDController(kP, kI, kD,
                new Constraints(ArmSubsystemConstants.MAX_SPEED_RAD_PER_SEC,
                        ArmSubsystemConstants.MAX_ACCEL_RAD_PER_SEC));
        controller.setTolerance(Units.degreesToRadians(2.0));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        feedforward = new ArmFeedforward(kS, kG, kV);

    }

    public void setPercent(double speed) {
        // SmartDashboard.putNumber("Percent", speed);
        // System.out.printf("\nPercent %.2f", speed);
        motor.set(speed);
    }

    public void setVoltage(double voltage) {
        // SmartDashboard.putNumber("Voltage", voltage);
        // System.out.printf("\nVoltage %.2f", voltage);
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
        motor.setVoltage(0.0);
    }

    @Override
    public void periodic() {

        if (homing) {
            setVoltage(12.0 * 0.15);
        } else {
            setVoltage(controller.calculate(getWrappedAngle())
                    + feedforward.calculate(getWrappedAngle(), controller.getSetpoint().velocity));

            SmartDashboard.putNumber("controller setpoint", controller.getSetpoint().velocity);
        }

        // setVoltage(RobotContainer.getJoy().getLeftX());

        // setPercent(RobotContainer.getJoy().getHID().getLeftX() * .8);

        SmartDashboard.putNumber("Position", getWrappedAngle());
        SmartDashboard.putNumber("Velocity", getVelocity());

        SmartDashboard.putNumber("Error", controller.getPositionError());

    }

    public Command setGoalCommand(double goal) {
        return new SetGoal(goal, this);
    }

    public Command armHomingCommand() {
        return new ArmHoming(this);
    }

}
