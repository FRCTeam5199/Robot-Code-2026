package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IndexerConstants {
    public static final int INDEXER_ROLLER_MOTOR_ID = 15;
    public static final double INDEXER_ROLLER_STATOR_CURRENT_LIMIT = 60;
    public static final double INDEXER_ROLLER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INDEXER_ROLLER_INVERTED = true;
    public static final boolean INDEXER_ROLLER_BRAKE = true;

    public static final Slot0Configs INDEXER_ROLLER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double INDEXER_ROLLER_ACCELERATION = 400;
    public static final double INDEXER_ROLLER_JERK = 4000;

    public static final double INDEXER_ROLLER_LOWER_TOLERANCE = 2;
    public static final double INDEXER_ROLLER_UPPER_TOLERANCE = 2;

    public static final double[][] INDEXER_ROLLER_GEAR_RATIO = {{1, 1}};
}
