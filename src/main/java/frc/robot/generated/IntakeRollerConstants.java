package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class IntakeRollerConstants {
    public static final int INTAKE_ROLLER_MOTOR_ID = 0;
    public static final double INTAKE_ROLLER_STATOR_CURRENT_LIMIT = 60;
    public static final double INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT = 60;
    public static final boolean INTAKE_ROLLER_INVERTED = false;
    public static final boolean INTAKE_ROLLER_BRAKE = true;

    public static final Slot0Configs INTAKE_ROLLER_SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final double INTAKE_ROLLER_ACCELERATION = 0;
    public static final double INTAKE_ROLLER_JERK = 0;

    public static final double INTAKE_ROLLER_LOWER_TOLERANCE = 2;
    public static final double INTAKE_ROLLER_UPPER_TOLERANCE = 2;

    public static final double[][] INTAKE_ROLLER_GEAR_RATIO = {{1, 1}};
}
