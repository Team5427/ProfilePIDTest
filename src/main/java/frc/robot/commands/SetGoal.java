package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetGoal extends CommandBase {

    private double setpoint;

    public SetGoal (double newSetpoint) {
        addRequirements(RobotContainer.getNeoMotor());

        setpoint = newSetpoint;
    }

    @Override
    public void initialize() {
        RobotContainer.getNeoMotor().setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.getNeoMotor().atGoal();
    }
    
}
