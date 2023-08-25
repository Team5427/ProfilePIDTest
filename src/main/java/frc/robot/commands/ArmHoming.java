package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmHoming extends CommandBase {

    private Timer timer;

    private Arm subsystem;

    public ArmHoming(Arm arm) {
        subsystem = arm;
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
    public void execute() {
        if (Math.abs(subsystem.getVelocity()) > .5) {
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.getHoming() && Math.abs(subsystem.getVelocity()) < .5 && timer.get() > .3;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("homed");
        subsystem.resetController();
        subsystem.setAngle(Constants.ArmSubsystemConstants.HARD_STOP_STATE_RAD);
        subsystem.setHoming(false);
        subsystem.stop();
        Timer.delay(1);
    }

}
