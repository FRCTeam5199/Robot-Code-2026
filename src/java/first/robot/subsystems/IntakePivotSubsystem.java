package first.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import first.robot.subsystems.templates.ArmSubsystem;
import first.robot.units.MechDegrees;
import first.robot.units.MotorRotations;

public class IntakePivotSubsystem extends ArmSubsystem{
    private static volatile IntakePivotSubsystem instance;

    public static IntakePivotSubsystem getInstance(){
        if(instance == null){
            synchronized (IntakePivotSubsystem.class){
                if(instance == null){
                    instance = new IntakePivotSubsystem();
                }
            }
        }
        return instance;
    }
    
    private MechDegrees POSITION_TOLERANCE = new MechDegrees(1);
    private final MotorRotations VELOCITY_TOLERANCE = new MotorRotations(10);

    public static final int MOTOR_ID = 17;
    public static final int ENCODER_ID = 99;

    public static final double STATOR_CURRENT_LIMIT = 40;
    public static final double SUPPLY_CURRENT_LIMIT = 40;
    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;

    public static final double DEPLOY = 95;
    public static final double UPAGITATE = 60;
    public static final double STOW = -1;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(10)
            .withKI(0)
            .withKD(0)
            .withKS(0.07)
            .withKG(.48)
            .withKV(.125)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(.0);

    public static final double VELOCITY = 50; //125
    public static final double ACCELERATION = 100; //200
    public static final double JERK = 1500; //3000

    public static final double MAGNET_OFFSET = .34912109375;
    public static final boolean ENCODER_DIRECTION = true;

    public static final double MIN = -2;
    public static final double MAX = 85;

    //    public static final double GEAR_RATIO = 48;
    public static final double[][] GEAR_RATIO = {{48, 1}};
    public static final double INTAKE_SENSOR_MECH_RATIO = 1d;

    public static final CANBus CANBUS = new CANBus("can_s0");
    private IntakePivotSubsystem(){
        configureMotor(MOTOR_ID, INVERTED, GEAR_RATIO, CANBUS, null, new MechDegrees(MAX), new MechDegrees(MIN * -1), STATOR_CURRENT_LIMIT, SUPPLY_CURRENT_LIMIT, NeutralModeValue.Brake, POSITION_TOLERANCE, VELOCITY_TOLERANCE, new MotorRotations(VELOCITY), new MotorRotations(ACCELERATION), new MotorRotations(JERK));
    }
}
