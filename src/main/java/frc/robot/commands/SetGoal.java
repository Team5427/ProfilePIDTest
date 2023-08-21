package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class SetGoal extends CommandBase {

    private double setpoint;
    private Arm arm;

    public SetGoal(double newSetpoint) {
        arm = RobotContainer.getArm();
        addRequirements(arm);

        setpoint = newSetpoint;
    }

    @Override
    public void initialize() {
        arm.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.atGoal();
    }

}
