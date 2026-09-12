package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IndexerConstants {
    public static final int UPPER_MOTOR_ID = 15;
    public static final int SECOND_UPPER_MOTOR_ID = 16;
    public static final int LOWER_MOTOR_ID = 25;

    public static final double UPPER_STATOR_CURRENT_LIMIT = 85;
    public static final double UPPER_SUPPLY_CURRENT_LIMIT = 60;
    public static final double LOWER_STATOR_CURRENT_LIMIT = 85;
    public static final double LOWER_SUPPLY_CURRENT_LIMIT = 60;

    public static final boolean UPPER_INVERTED = false;
    public static final boolean SECONDARY_UPPER_INVERTED = true;
    public static final boolean LOWER_INVERTED = false;

    public static final boolean UPPER_BRAKE = false;
    public static final boolean LOWER_BRAKE = false;

    public static final double UPPER_INDEXER_SPEED = 25;
    public static final double LOWER_INDEXER_SPEED = UPPER_INDEXER_SPEED * 2.5;

    public static final Slot0Configs UPPER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(.40156)
            .withKG(0)
            .withKV(.12621)
            .withKA(0.15)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final Slot0Configs LOWER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(.40156)
            .withKG(0)
            .withKV(.12621)
            .withKA(0.15)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double ACCELERATION = 400;
    public static final double JERK = 4000;

    public static final double LOWER_TOLERANCE = 8d;
    public static final double UPPER_TOLERANCE = 8d;

    public static final double[][] GEAR_RATIO = {{1, 1}};
    public static final double SCALE_FACTOR = (1d);

    public static final CANBus canbus = new CANBus("rio");
}
