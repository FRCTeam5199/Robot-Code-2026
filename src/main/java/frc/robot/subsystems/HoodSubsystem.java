package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.subsystems.templates.ArmSubsystem;
import frc.robot.units.MechDegrees;
import frc.robot.units.MotorRotations;

public class HoodSubsystem extends ArmSubsystem{
    private static volatile HoodSubsystem instance;

    public static HoodSubsystem getInstance(){
        if(instance == null){
            synchronized (HoodSubsystem.class){
                if(instance == null){
                    instance = new HoodSubsystem();
                }
            }
        }
        return instance;
    }

    private MechDegrees POSITION_TOLERANCE = new MechDegrees(1.25);
    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(0.1);

    public static final int MOTOR_ID = 19;
    public static final int ENCODER_ID = 99;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(50)
            .withKI(0)
            .withKD(0)
            .withKS(.47)
            .withKG(0)
            .withKV(.087719298245614)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0);

    public static final double VELOCITY = 100;
    public static final double ACCELERATION = 200;
    public static final double JERK = 4000;


    public static final MechDegrees MIN = new MechDegrees(0);
    public static final MechDegrees MAX = new MechDegrees(25);

    public static final double[][] GEAR_RATIO = {{60, 11}, {146, 10}}; //.01255707762557077625570776255708

    public static final CANBus CANBUS = new CANBus("can_s2");

    private HoodSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CANBUS, SLOT0_CONFIGS, MAX, MIN, STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, NeutralModeValue.Brake, POSITION_TOLERANCE, VELOCITY_TOLERANCE, new MotorRotations(VELOCITY), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
    }
}