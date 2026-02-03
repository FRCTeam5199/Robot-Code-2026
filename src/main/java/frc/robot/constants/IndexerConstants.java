package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 24;
    public static final double INDEXER_STATOR_CURRENT_LIMIT = 60;
    public static final double INDEXER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INDEXER_INVERTED = true;
    public static final boolean INDEXER_BRAKE = true;

    public static final Slot0Configs INDEXER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKS(0.2)
            .withKG(0)
            .withKV(0.11111111111111111111111)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double INDEXER_ACCELERATION = 400;
    public static final double INDEXER_JERK = 4000;

    public static final double INDEXER_LOWER_TOLERANCE = 2;
    public static final double INDEXER_UPPER_TOLERANCE = 2;

    public static final double[][] INDEXER_GEAR_RATIO = {{1, 1}};
}
