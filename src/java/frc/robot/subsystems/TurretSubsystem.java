package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.subsystems.templates.RollerSubsystem;
import frc.robot.units.MotorRotations;

public class TurretSubsystem extends RollerSubsystem{
    private static volatile TurretSubsystem instance;

    public static TurretSubsystem getInstance(){
        if(instance == null){
            synchronized (TurretSubsystem.class){
                if(instance == null){
                    instance = new TurretSubsystem();
                }
            }
        }
        return instance;
    }

    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(10);

    public static final int MOTOR_ID = 21;
    public static final int ENCODER_ID = 23;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(.3)
            .withKI(0)
            .withKD(0.18)
            .withKS(.34)
            .withKG(0)
            .withKV(.0943396226415094339622641509434)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double VELOCITY = 200;
    public static final double ACCELERATION = 400;
    public static final double JERK = 8000;

    public static final double LOWER_TOLERANCE = 10;
    public static final double UPPER_TOLERANCE = 10;

    public static final double[][] GEAR_RATIO = {{50, 12}, {100, 10}};

    public static final boolean CCW_POSITIVE = true;
    public static final double ABSOLUTE_DISCONTINUITY_POINT = .5;

    public static final double MIN = -260d;
    public static final double MAX = 270d;

    public static final double INDEXING_TIME = .2;

    public static final CANBus CANBUS = new CANBus("can_s2");

    private TurretSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CANBUS, SLOT0_CONFIGS, null, null, STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, VELOCITY_TOLERANCE, new MotorRotations(400), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
        // configureMotorFollower(MOTOR_ID, 1, true);
    }
}
