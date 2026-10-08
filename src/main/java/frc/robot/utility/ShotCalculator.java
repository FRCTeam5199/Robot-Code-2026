// package frc.robot.utility;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.constants.Constants;
// import frc.robot.constants.TurretConstants;
// import frc.robot.subsystems.HoodSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TurretSubsystem;

// public class ShotCalculator {
//     private static ShotCalculator shotCalculator;
//     //    private final LinearFilter turretAngleFilter =
// //            LinearFilter.movingAverage((int) (0.1 / 0.02));
//     private Pose2d futureTurretPosition, futureTurretPositionPhaseDelayed;
//     private double turretAngle, turretAnglePhaseDelayed;
//     private double lastTurretAngle, lastTurretAnglePhaseDelayed;
//     private Rotation2d lastTurretRotation, lastTurretRotationPhaseDelayed, turretRotation, turretRotationPhaseDelayed;
//     private double turretVelocity, turretVelocityPhaseDelayed;
//     private InterpolatingDoubleTreeMap hoodLookupTable;
//     private InterpolatingDoubleTreeMap shooterSpeedLookupTable;
//     private InterpolatingDoubleTreeMap timeOfFlightLookupTable;
//     private InterpolatingDoubleTreeMap shuttleHoodLookupTable;
//     private InterpolatingDoubleTreeMap shuttleShooterSpeedLookupTable;
//     private InterpolatingDoubleTreeMap shuttleTimeOfFlightLookupTable;
//     private double hoodAnglePhaseDelayed, lastHoodAnglePhaseDelayed, hoodAngle, lastHoodAngle, hoodVelocityPhaseDelayed, hoodVelocity;
//     private double shooterSpeed, shooterSpeedPhaseDelayed;
//     private TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
//     private HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
//     private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
//     private double futureTurretToTargetDistance, futureTurretToTargetDistancePhaseDelayed = 0;
//     private static final double CONVERGENCE_EPSILON_METERS = 0.005;

//     private ShotCalculator() {
//         //Motor Rotations = degrees / 360 / .01255707762557077625570776255708
//         //Degrees = motorRot * 360 * .01255707762557077625570776255708
//         hoodLookupTable = new InterpolatingDoubleTreeMap();
//         shooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
//         timeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

//         shuttleHoodLookupTable = new InterpolatingDoubleTreeMap();
//         shuttleShooterSpeedLookupTable = new InterpolatingDoubleTreeMap();
//         shuttleTimeOfFlightLookupTable = new InterpolatingDoubleTreeMap();

//         lastTurretAnglePhaseDelayed = turretSubsystem.getDegrees();
//     }


//     public static ShotCalculator getInstance() {
//         if (shotCalculator == null) shotCalculator = new ShotCalculator();
//         return shotCalculator;
//     }

//     public void periodic() {
//         if (RobotContainer.getPose() != null) {
//             Pose2d estimatedPose = RobotContainer.getPose();
//             Pose2d estimatedPosePhaseDelayed =
//                     estimatedPose.exp(
//                             new Twist2d(
//                                     RobotContainer.getSpeeds().vxMetersPerSecond * Constants.PHASE_DELAY
//                                             + RobotContainer.getAccelerationX() * Constants.ACCELERATION_PHASE_DELAY,
//                                     RobotContainer.getSpeeds().vyMetersPerSecond * Constants.PHASE_DELAY
//                                             + RobotContainer.getAccelerationY() * Constants.ACCELERATION_PHASE_DELAY,
//                                     RobotContainer.getSpeeds().omegaRadiansPerSecond * Constants.PHASE_DELAY
//                                             + RobotContainer.getAccelerationOmega() * Constants.ACCELERATION_PHASE_DELAY));

//             Translation2d target = getCurrentTarget();
//             ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//                     RobotContainer.getSpeeds(), RobotContainer.getPose().getRotation());

//             Pair<Pose2d, Double> turretPhaseDelayed = createFutureTurretPose(estimatedPosePhaseDelayed, robotRelativeSpeeds, target);
//             Pair<Pose2d, Double> turret = createFutureTurretPose(estimatedPose, robotRelativeSpeeds, target);

//             futureTurretPositionPhaseDelayed = turretPhaseDelayed.getFirst();
//             futureTurretToTargetDistancePhaseDelayed = turretPhaseDelayed.getSecond();

//             futureTurretPosition = turret.getFirst();
//             futureTurretToTargetDistance = turret.getSecond();

//             if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
//                 hoodAnglePhaseDelayed = hoodLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
//                 hoodAngle = hoodLookupTable.get(futureTurretToTargetDistance);
//             } else {
//                 hoodAnglePhaseDelayed = shuttleHoodLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
//                 hoodAngle = shuttleHoodLookupTable.get(futureTurretToTargetDistance);
//             }
//             if (Double.isNaN(lastHoodAnglePhaseDelayed)) lastHoodAnglePhaseDelayed = hoodAnglePhaseDelayed;
//             hoodVelocityPhaseDelayed = (hoodAnglePhaseDelayed - lastHoodAnglePhaseDelayed) / .005;

//             if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
//             hoodVelocity = (hoodAngle - lastHoodAngle) / .005;

//             turretRotationPhaseDelayed = target.minus(futureTurretPositionPhaseDelayed.getTranslation()).getAngle();

//             if (lastTurretRotationPhaseDelayed == null) lastTurretRotationPhaseDelayed = turretRotationPhaseDelayed;
//             turretVelocityPhaseDelayed = turretRotationPhaseDelayed
//                     .minus(lastTurretRotationPhaseDelayed).getDegrees() / .005;

//             // Sets turret angle and then adjusts it based on robot's rotation in relation to target
//             turretAnglePhaseDelayed = turretRotationPhaseDelayed.getDegrees();
//             turretAnglePhaseDelayed -= futureTurretPositionPhaseDelayed.getRotation().getDegrees();

//             //Wraps to within bounds
//             double[] turretAngles = new double[]{turretAnglePhaseDelayed - 360,
//                     turretAnglePhaseDelayed, turretAnglePhaseDelayed + 360};
//             int smallestValidDistanceIndex = -1;
//             double smallestDistance = 99999d;
//             for (int i = 0; i < 3; i++) {
//                 double angle = turretAngles[i];
//                 if (angle <= TurretConstants.MIN || angle >= TurretConstants.MAX) continue;
//                 if (Math.abs(angle - turretSubsystem.getDegrees()) < smallestDistance) {
//                     smallestDistance = Math.abs(angle - turretSubsystem.getDegrees());
//                     smallestValidDistanceIndex = i;
//                 }
//             }
//             if (smallestValidDistanceIndex == -1) turretAnglePhaseDelayed = turretAngles[1];
//             else turretAnglePhaseDelayed = turretAngles[smallestValidDistanceIndex];

//             turretVelocityPhaseDelayed -= (Math.toDegrees(RobotContainer.getSpeeds().omegaRadiansPerSecond));

//             //Turret Angle Calculations
//             turretRotation = target.minus(futureTurretPosition.getTranslation()).getAngle();

//             if (lastTurretRotation == null) lastTurretRotation = turretRotation;
//             turretVelocity = turretRotation
//                     .minus(lastTurretRotation).getDegrees() / .005;

//             // Sets turret angle and then adjusts it based on robot's rotation in relation to target
//             turretAngle = turretRotation.getDegrees();
//             turretAngle -= RobotContainer.getPose().getRotation().getDegrees();

//             //Wraps to within bounds
//             turretAngles = new double[]{turretAngle - 360, turretAngle, turretAngle + 360};
//             smallestValidDistanceIndex = -1;
//             smallestDistance = 99999d;
//             for (int i = 0; i < 3; i++) {
//                 double angle = turretAngles[i];
//                 if (angle <= TurretConstants.MIN || angle >= TurretConstants.MAX) continue;
//                 if (Math.abs(angle - turretSubsystem.getDegrees()) < smallestDistance) {
//                     smallestDistance = Math.abs(angle - turretSubsystem.getDegrees());
//                     smallestValidDistanceIndex = i;
//                 }
//             }
//             if (smallestValidDistanceIndex == -1) turretAngle = turretAngles[1];
//             else turretAngle = turretAngles[smallestValidDistanceIndex];

//             turretVelocity -= (Math.toDegrees(RobotContainer.getSpeeds().omegaRadiansPerSecond));

//             // Based on Future Pose
//             if (RobotContainer.getShotMode() == ShotMode.SHOOTING) {
//                 shooterSpeedPhaseDelayed = shooterSpeedLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
//                 shooterSpeed = shooterSpeedLookupTable.get(futureTurretToTargetDistance);
//             } else {
//                 shooterSpeedPhaseDelayed = shuttleShooterSpeedLookupTable.get(futureTurretToTargetDistancePhaseDelayed);
//                 shooterSpeed = shuttleShooterSpeedLookupTable.get(futureTurretToTargetDistance);
//             }
//         }

//         lastTurretRotation = turretRotation;
//         lastTurretRotationPhaseDelayed = turretRotationPhaseDelayed;
//         lastHoodAnglePhaseDelayed = hoodAnglePhaseDelayed;
//         lastHoodAngle = hoodAngle;


//         //Offset for left corner
//         if (Robot.getAlliance() != null) {
//             if (futureTurretToTargetDistance >= 4) {
//                 if ((Robot.getAlliance().equals(DriverStation.Alliance.Red)
//                         && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
//                         || (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
//                         && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
//                     turretAngle += .25;
//                     turretAnglePhaseDelayed += .25;
//                     shooterSpeed += .25;
//                     shooterSpeedPhaseDelayed += .25;
//                 }
//             }
//             if (futureTurretToTargetDistance >= 4.5) {
//                 if ((Robot.getAlliance().equals(DriverStation.Alliance.Red)
//                         && futureTurretPosition.getY() < Constants.RED_HUB_CENTER.getY())
//                         || (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
//                         && futureTurretPosition.getY() > Constants.BLUE_HUB_CENTER.getY())) {
//                     turretAngle += 1d;
//                     turretAnglePhaseDelayed += 1d;
//                     shooterSpeed += .75;
//                     shooterSpeedPhaseDelayed += .75;
//                 }
//             }
//             if (futureTurretToTargetDistance >= 4.65) {
//                 if ((Robot.getAlliance().equals(DriverStation.Alliance.Red)
//                         && futureTurretPosition.getY() > Constants.RED_HUB_CENTER.getY())
//                         || (Robot.getAlliance().equals(DriverStation.Alliance.Blue)
//                         && futureTurretPosition.getY() < Constants.BLUE_HUB_CENTER.getY())) {
//                     shooterSpeed += .75;
//                     shooterSpeedPhaseDelayed += .75;
//                 }
//             }
//         }

//         turretSubsystem.updateGoalPosition(getTurretAnglePhaseDelayed(), getTurretVelocityPhaseDelayed());
//         RobotContainer.getShooterControlAuto().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
//         RobotContainer.getShooterControlAutoRB().setGoal(shotCalculator.getShooterSpeedPhaseDelayed());
//         hoodSubsystem.updateGoalPosition(getHoodAnglePhaseDelayed(), getHoodVelocityPhaseDelayed());

//     }


//     public Pair<Pose2d, Double> createFutureTurretPose(Pose2d pose2d, ChassisSpeeds fieldRelativeVelocity, Translation2d target) {
//         Pose2d turretPosition = pose2d.transformBy(Constants.ROBOT_TO_TURRET);

//         // Sets distance values for lookup tables based on ShotMode
//         double baseX = turretPosition.getX();
//         double baseY = turretPosition.getY();
//         double targetX = target.getX();
//         double targetY = target.getY();
//         double turretToTargetDistance = Math.hypot(targetX - baseX, targetY - baseY);

//         //Robot's current velocity
//         double robotAngleRadians = pose2d.getRotation().getRadians();

//         //Turret's current velocity imparted from robot
//         double turretVelocityX =
//                 fieldRelativeVelocity.vxMetersPerSecond
//                         + fieldRelativeVelocity.omegaRadiansPerSecond
//                         * (-Constants.ROBOT_TO_TURRET.getY() * Math.cos(robotAngleRadians)
//                         - Constants.ROBOT_TO_TURRET.getX() * Math.sin(robotAngleRadians));
//         double turretVelocityY =
//                 fieldRelativeVelocity.vyMetersPerSecond
//                         + fieldRelativeVelocity.omegaRadiansPerSecond
//                         * (Constants.ROBOT_TO_TURRET.getX() * Math.cos(robotAngleRadians)
//                         - Constants.ROBOT_TO_TURRET.getY() * Math.sin(robotAngleRadians));

//         //Account for velocity
//         double timeOfFlight;
//         double futureX = baseX, futureY = baseY;
//         double lookaheadTurretToTargetDistance = turretToTargetDistance;

// //        Iterate to converge on a final future position
//         for (int i = 0; i < 10; i++) {
//             timeOfFlight = timeOfFlightLookupTable.get(lookaheadTurretToTargetDistance);
//             futureX = baseX + turretVelocityX * timeOfFlight;
//             futureY = baseY + turretVelocityY * timeOfFlight;

//             double newDistance = Math.hypot(targetX - futureX, targetY - futureY);
//             boolean converged = Math.abs(newDistance - lookaheadTurretToTargetDistance) < CONVERGENCE_EPSILON_METERS;
//             lookaheadTurretToTargetDistance = newDistance;
//             if (converged) break;
//         }

//         Pose2d futureTurretPosition = new Pose2d(futureX, futureY, turretPosition.getRotation());
//         return new Pair<>(futureTurretPosition, lookaheadTurretToTargetDistance);
//     }

//     public double getHoodAngle() {
//         return hoodAngle;
//     }

//     public double getHoodVelocity() {
//         return hoodVelocity;
//     }

//     public double getHoodAnglePhaseDelayed() {
//         return hoodAnglePhaseDelayed;
//     }

//     public double getHoodVelocityPhaseDelayed() {
//         return hoodVelocityPhaseDelayed;
//     }

//     public double getShooterSpeed() {
//         return shooterSpeed;
//     }

//     public double getShooterSpeedPhaseDelayed() {
//         return shooterSpeedPhaseDelayed;
//     }

//     public double getTurretAngle() {
//         return turretAngle;
//     }

//     public double getTurretVelocity() {
//         return turretVelocity;
//     }

//     public double getTurretAnglePhaseDelayed() {
//         return turretAnglePhaseDelayed;
//     }

//     public double getTurretVelocityPhaseDelayed() {
//         return turretVelocityPhaseDelayed;
//     }

//     public double getTurretToTargetDistance() {
//         return futureTurretToTargetDistance;
//     }

//     public Pose2d getFutureTurretPosition() {
//         return futureTurretPosition;
//     }

//     public Pose2d getFutureTurretPositionPhaseDelayed() {
//         return futureTurretPositionPhaseDelayed;
//     }

//     public boolean isWithinBounds() {
//         if (RobotContainer.getShotMode() != ShotMode.SHOOTING) return true;
//         return futureTurretToTargetDistance >= (.997 + Constants.HUB_RADIUS)
//                 && futureTurretToTargetDistance <= (5d + Constants.HUB_RADIUS);
//     }

//     private Translation2d getCurrentTarget() {
//         ShotMode mode = RobotContainer.getShotMode();
//         if (mode == ShotMode.SHOOTING) {
//             return AllianceFlipper.getCorrectAlliance(Constants.BLUE_HUB_CENTER, Constants.RED_HUB_CENTER);
//         } else if (mode == ShotMode.SHUTTLING_LEFT) {
//             return AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_LEFT_CORNER, Constants.RED_SHUTTLE_LEFT_CORNER);
//         } else {
//             return AllianceFlipper.getCorrectAlliance(Constants.BLUE_SHUTTLE_RIGHT_CORNER, Constants.RED_SHUTTLE_RIGHT_CORNER);
//         }
//     }
// }

