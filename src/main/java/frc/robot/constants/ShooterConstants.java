package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ShooterConstants {
    public static final int MOTOR_ID = 22;
    public static final int SECOND_MOTOR_ID = 26;
    public static final int THIRD_MOTOR_ID = 27;
    public static final int FOURTH_MOTOR_ID = 25;
    public static final double STATOR_CURRENT_LIMIT = 65;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final boolean SECOND_INVERTED = false;
    public static final boolean THIRD_INVERTED = true;
    public static final boolean FOURTH_INVERTED = true;
    public static final boolean BRAKE = false;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.0068213)
            .withKI(0)
            .withKD(0)
            .withKS(.38)
            .withKG(0)
            .withKV(.1270648030495)
            .withKA(0.0045384)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 120000;
    public static final double JERK = 80000000;

    public static final double LOWER_TOLERANCE = 4;
    public static final double UPPER_TOLERANCE = 17;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final CANBus CANBUS = new CANBus("Canivore");
}
