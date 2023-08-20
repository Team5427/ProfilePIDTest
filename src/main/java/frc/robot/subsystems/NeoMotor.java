package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoSubsystemConstants;

public class NeoMotor extends SubsystemBase {

    private CANSparkMax neo;
    private RelativeEncoder encoder;

    private ProfiledPIDController controller;
    private ArmFeedforward feedforward;

    private double setpoint_rad = 0.0;

    private boolean homing = false;

    public NeoMotor () {

        neo = new CANSparkMax(NeoSubsystemConstants.NEO_ID, MotorType.kBrushless);
        encoder = neo.getEncoder();

        encoder.setPositionConversionFactor(NeoSubsystemConstants.POS_CONV_FACTOR);
        encoder.setVelocityConversionFactor(NeoSubsystemConstants.VELOCITY_CONV_FACTOR);
        neo.setSmartCurrentLimit(NeoSubsystemConstants.CURRENT_LIMIT);

        controller = new ProfiledPIDController(0.0, 0.0, 0.0, 
            new Constraints(NeoSubsystemConstants.MAX_SPEED_RAD_PER_SEC, NeoSubsystemConstants.MAX_ACCEL_RAD_PER_SEC)
        );
        controller.setTolerance(Units.degreesToRadians(2.0));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        feedforward = new ArmFeedforward(0.01, 0.04, 1.36);

    }

    public void setPercent (double speed) { neo.set(speed); }

    public void setVoltage (double voltage) { neo.setVoltage(voltage); }

    public void setSetpoint (double setpoint) { 
        this.setpoint_rad = setpoint; 
        controller.setGoal(new State(setpoint_rad, 0.0));
    }

    public void setAngle (double angle) { encoder.setPosition(angle); }

    public void setHoming (boolean homing) { this.homing = homing; } 

    public double getAngle () { return encoder.getPosition(); }

    public double getVelocity () { return encoder.getVelocity(); }

    public double getWrappedAngle () { return MathUtil.angleModulus(getAngle()); }

    public Rotation2d getRotation2d () { return new Rotation2d(getAngle()); }

    public boolean getHoming () { return homing; }

    public boolean atGoal () { return controller.atGoal(); }

    public void resetController () { controller.reset(new State(getWrappedAngle(), getVelocity())); }

    public void stop () { neo.set(0.0); }

    @Override
    public void periodic() {

        if (homing) {
            neo.set(0.05);
        } else {
            neo.setVoltage(controller.calculate(getWrappedAngle()) + feedforward.calculate(getWrappedAngle(), controller.getSetpoint().velocity));
        }

    }
    
}
