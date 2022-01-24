package team3647.frc2022.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import team3647.frc2022.subsystems.vision.VisionController;
import team3647.lib.vision.MultiTargetTracker;

public class FlightDeck {
    private final RobotTracker robotTracker;
    private final VisionController vision;
    private final MultiTargetTracker targetTracker;
    private final Translation2d kRobotToTurretFixed;
    private final Translation2d kTurretToCamFixed;
    private final Rotation2d kCamPitchFixed;

    public FlightDeck(
            RobotTracker robotTracker,
            VisionController vision,
            MultiTargetTracker targetTracker,
            Translation2d kRobotToTurretFixed,
            Translation2d kTurretToCamFixed,
            Rotation2d kCamPitchFixed) {
        this.robotTracker = robotTracker;
        this.vision = vision;
        this.targetTracker = targetTracker;
        this.kRobotToTurretFixed = kRobotToTurretFixed;
        this.kTurretToCamFixed = kTurretToCamFixed;
        this.kCamPitchFixed = kCamPitchFixed;
    }

    public void update() {
        this.robotTracker.update();
    }

    private final Rotation2d camPitch = Rotation2d.fromDegrees(45);
    private final Translation2d robotToTurretFixed = new Translation2d(Units.inchesToMeters(7), 0);
    private final Translation2d turretToCam = new Translation2d(Units.inchesToMeters(7), 0);
}
