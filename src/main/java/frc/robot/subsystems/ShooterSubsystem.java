package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.subsystems.templates.RollerSubsystem;
import frc.robot.units.MotorRotations;

public class ShooterSubsystem extends RollerSubsystem{
    private static volatile ShooterSubsystem instance;

    public static ShooterSubsystem getInstance(){
        if(instance == null){
            synchronized (ShooterSubsystem.class){
                if(instance == null){
                    instance = new ShooterSubsystem();
                }
            }
        }
        return instance;
    }

    public static final int MOTOR_ID = 22;
    public static final int SECOND_MOTOR_ID = 20;
    public static final double STATOR_CURRENT_LIMIT = 100;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean SECOND_INVERTED = true;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.0068213)
            .withKI(0)
            .withKD(0)
            .withKS(.27715)
            .withKG(0)
            .withKV(.12194)
            .withKA(0.0045384)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 80000;
    public static final double JERK = 80000000;

    public static final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(2);

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final CANBus CANBUS = new CANBus("can_s2");

    private ShooterSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CANBUS, SLOT0_CONFIGS, null, null, STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, VELOCITY_TOLERANCE, new MotorRotations(400), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
        configureMotorFollower(MOTOR_ID, SECOND_MOTOR_ID, SECOND_INVERTED);
    }
}
