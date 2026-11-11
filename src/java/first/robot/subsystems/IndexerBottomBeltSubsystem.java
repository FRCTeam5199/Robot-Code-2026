package first.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import first.robot.subsystems.templates.RollerSubsystem;
import first.robot.units.MotorRotations;

public class IndexerBottomBeltSubsystem extends RollerSubsystem{
    private static volatile IndexerBottomBeltSubsystem instance;

    public static IndexerBottomBeltSubsystem getInstance(){
        if(instance == null){
            synchronized (IndexerBottomBeltSubsystem.class){
                if(instance == null){
                    instance = new IndexerBottomBeltSubsystem();
                }
            }
        }
        return instance;
    }

    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(0.01);

    public static final int LOWER_MOTOR_ID = 25;

    public static final double LOWER_STATOR_CURRENT_LIMIT = 85;
    public static final double LOWER_SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean LOWER_INVERTED = false;

    public static final boolean LOWER_BRAKE = false;

    public static final double UPPER_INDEXER_SPEED = 20;
    public static final double UPPER_TO_LOWER_RATIO = 3d; //2.5 is 1:1
    public static final double LOWER_INDEXER_SPEED = UPPER_INDEXER_SPEED * UPPER_TO_LOWER_RATIO;

    public static final Slot0Configs LOWER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.0014654)
            .withKI(0)
            .withKD(0)
            .withKS(.43)
            .withKG(0)
            .withKV(.1)
            .withKA(0.15)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 8d;


    public static final double[][] GEAR_RATIO = {{1, 1}};
    public static final double SCALE_FACTOR = (1d);

    public static final CANBus CANBUS = new CANBus("can_s0");

    private IndexerBottomBeltSubsystem(){
        configureMotor(LOWER_MOTOR_ID, LOWER_INVERTED, GEAR_RATIO, CANBUS, LOWER_SLOT0_CONFIGS, null, null, LOWER_STATOR_CURRENT_LIMIT, LOWER_SUPPLY_CURRENT_LIMIT, VELOCITY_TOLERANCE, new MotorRotations(400), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
    }
}
