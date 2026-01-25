package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class HopperConstants {
    public static final int HOPPER_MOTOR_ID = 17;
    public static final double HOPPER_STATOR_CURRENT_LIMIT = 60;
    public static final double HOPPER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean HOPPER_INVERTED = true;
    public static final boolean HOPPER_BRAKE = true;

    public static final Slot0Configs HOPPER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0.25)
            .withKI(0)
            .withKD(0)
            .withKS(0.9)
            .withKG(0)
            .withKV(0.08787346221441124780316344463972)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double HOPPER_ACCELERATION = 400;
    public static final double HOPPER_JERK = 4000;

    public static final double HOPPER_LOWER_TOLERANCE = 2;
    public static final double HOPPER_UPPER_TOLERANCE = 2;

    public static final double[][] HOPPER_GEAR_RATIO = {{1, 1}};
}
