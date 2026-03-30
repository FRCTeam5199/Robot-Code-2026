package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class HoodConstants {
    public static final int MOTOR_ID = 19;
    public static final int ENCODER_ID = 99;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(.5)
            .withKG(0)
            .withKV(.08264462809917355371900826446281)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0);

    public static final double VELOCITY = 150;
    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = .5;
    public static final double UPPER_TOLERANCE = .5;

    public static final double MAGNET_OFFSET = -0.14111328125;
    public static final boolean IS_CCW_POS = false;

    public static final double MIN = 0;
    public static final double MAX = 29;

    public static final double MOTOR_TO_SENSOR_GEAR_RATIO = 8d;
    public static final double[][] GEAR_RATIO = {{60, 11}, {146, 10}}; //.01255707762557077625570776255708
    public static final double SENSOR_TO_MECH_GEAR_RATIO = 18d;

    public static final double ABSOLUTE_DISCONTINUITY_POINT = .5;

    public static final CANBus CANBUS = new CANBus("Shooter");
}


