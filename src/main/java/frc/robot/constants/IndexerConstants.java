package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IndexerConstants {
    public static final int MOTOR_ID = 25;
    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKS(0.2)
            .withKG(0)
            .withKV(0.11111111111111111111111)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 5;
    public static final double UPPER_TOLERANCE = 5;

    public static final double[][] GEAR_RATIO = {{1, 1}};

    public static final double INDEXING_SPEED = 90;
    public static final double IDLING_SPEED = -30;

    public static final CANBus canbus = new CANBus("rio");
}
