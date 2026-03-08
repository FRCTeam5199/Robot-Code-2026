package frc.robot.subsystems;

import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.templates.TemplateSubsystem;
import frc.robot.utility.Type;

public class IndexerSubsystem extends TemplateSubsystem {
    private static IndexerSubsystem indexerSubsystem;

    private IndexerSubsystem() {
        super(Type.ROLLER, IndexerConstants.MOTOR_ID,
                0, IndexerConstants.ACCELERATION,
                IndexerConstants.JERK,
                IndexerConstants.LOWER_TOLERANCE,
                IndexerConstants.UPPER_TOLERANCE,
                IndexerConstants.GEAR_RATIO, "Indexer", true);

        configureMotor(IndexerConstants.INVERTED, IndexerConstants.BRAKE,
                IndexerConstants.SUPPLY_CURRENT_LIMIT,
                IndexerConstants.STATOR_CURRENT_LIMIT,
                IndexerConstants.SLOT0_CONFIGS);
    }

    public static IndexerSubsystem getInstance() {
        if (indexerSubsystem == null) {
            indexerSubsystem = new IndexerSubsystem();
        }
        return indexerSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();
        //    System.out.println("Indexer Velocity: " + getMotorVelocity());
    }
}
