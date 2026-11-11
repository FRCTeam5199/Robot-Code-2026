package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.subsystems.templates.RollerSubsystem;
import frc.robot.units.MotorRotations;

public class IntakeRollerSubsystem extends RollerSubsystem{
    private static volatile IntakeRollerSubsystem instance;

    public static IntakeRollerSubsystem getInstance(){
        if(instance == null){
            synchronized (IntakeRollerSubsystem.class){
                if(instance == null){
                    instance = new IntakeRollerSubsystem();
                }
            }
        }
        return instance;
    }

    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(2);

    public static final int MOTOR_ID = 18;
    public static final int FOLLOWER_MOTOR_ID = 24;
    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = false;
    public static final boolean FOLLOWER_INVERTED = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(.0012331)
            .withKI(0)
            .withKD(0)
            .withKS(0.055467)
            .withKG(0)
            .withKV(.11466)
            .withKA(0.1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 2;
    public static final double UPPER_TOLERANCE = 2;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final double INTAKE_SPEED = 100; //116

    public static final CANBus CAN_BUS = new CANBus("can_s0");

    private IntakeRollerSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CAN_BUS, SLOT0_CONFIGS, null, null, STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, VELOCITY_TOLERANCE, new MotorRotations(400), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
        configureMotorFollower(MOTOR_ID, FOLLOWER_MOTOR_ID, FOLLOWER_INVERTED);
    }
}
