package first.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import first.robot.subsystems.templates.RollerSubsystem;
import first.robot.units.MotorRotations;

public class HopperSubsystem extends RollerSubsystem{
    private static volatile HopperSubsystem instance;

    public static HopperSubsystem getInstance(){
        if(instance == null){
            synchronized (HopperSubsystem.class){
                if(instance == null){
                    instance = new HopperSubsystem();
                }
            }
        }
        return instance;
    }

    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(50);

    public static final int MOTOR_ID = 14;
    public static final double STATOR_CURRENT_LIMIT = 50;
    public static final double SUPPLY_CURRENT_LIMIT = 50;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.1)
            .withKI(0)
            .withKD(0)
            .withKS(.34)
            .withKG(0)
            .withKV(.12121212121212121212121212121212)
            .withKA(0.1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double[][] GEAR_RATIO = {{1, 2.5}};

    public static final double INDEXING_SPEED = 50;
    public static final double REVERSE_SPEED = -100;
    public static final double IDLING_SPEED = -5;

    public static final CANBus CANBUS = new CANBus("can_s0");

    private HopperSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CANBUS, SLOT0_CONFIGS, null, null, STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, VELOCITY_TOLERANCE, new MotorRotations(400), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
    }
}
