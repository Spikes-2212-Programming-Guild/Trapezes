package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;

public class Elevator extends MotoredGenericSubsystem {

    private static Elevator instance;

    private final RelativeEncoder encoder;

    public static Elevator getInstance() {
        if (instance == null) {
            CANSparkMax sparkMax = new CANSparkMax(RobotMap.CAN.ELEVATOR_SPARK, MotorType.kBrushless);
            instance = new Elevator("elevator", sparkMax, sparkMax.getEncoder());
        }
        return instance;
    }

    private Elevator(String namespaceName, CANSparkMax sparkMax, RelativeEncoder encoder) {
        super(namespaceName, sparkMax);
        this.encoder = encoder;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}
