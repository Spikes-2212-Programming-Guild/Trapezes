package frc.robot;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class MoveSubsystemTrapezically extends CommandBase {

    private final GenericSubsystem subsystem;

    private final PIDController pidController;
    private final FeedForwardController ffController;
    private final Supplier<Double> source;
    private final TrapezoidProfile trapezoidProfile;
    private final Timer timer;

    public MoveSubsystemTrapezically(GenericSubsystem subsystem, Supplier<Double> source, PIDSettings pidSettings,
                                     FeedForwardSettings ffSettings, TrapezoidProfile trapezoidProfile) {
        this.subsystem = subsystem;
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        this.pidController.setTolerance(pidSettings.getTolerance());
        this.ffController = new FeedForwardController(ffSettings, FeedForwardController.DEFAULT_PERIOD);
        this.source = source;
        this.trapezoidProfile = trapezoidProfile;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        State state = trapezoidProfile.calculate(timer.get());
        subsystem.move(pidController.calculate(
                source.get(), state.velocity) + ffController.calculate(state.velocity));
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return trapezoidProfile.isFinished(timer.get());
    }
}
