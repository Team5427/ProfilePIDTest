package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NeoMotor;

public class NeoMotorHoming extends CommandBase {

    private Timer timer;

    private NeoMotor subsystem;

    public NeoMotorHoming () {
        subsystem = RobotContainer.getNeoMotor();
        addRequirements(subsystem);
        
        timer = new Timer();
    }

    @Override
    public void initialize() {
        subsystem.setHoming(true);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return subsystem.getHoming() && Math.abs(subsystem.getVelocity()) < .25 && timer.get() > .5;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.resetController();
        subsystem.setHoming(false);
    }
    
}
