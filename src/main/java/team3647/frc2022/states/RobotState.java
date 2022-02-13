package team3647.frc2022.states;

public class RobotState {
    public static final RobotState STOPPED = new RobotState();
    public static final RobotState CLIMB =
            new RobotState(
                    ShooterState.STOPPED, TurretState.CLIMB, FeederState.STOP, ClimberState.CLIMB);

    public ShooterState shooterState;
    public TurretState turretState;
    public FeederState feederState;
    public ClimberState climberState;

    public RobotState(
            ShooterState shooterState,
            TurretState turretState,
            FeederState feederState,
            ClimberState climberState) {
        this.shooterState = shooterState;
        this.turretState = turretState;
        this.feederState = feederState;
        this.climberState = climberState;
    }

    public RobotState() {
        this(ShooterState.STOPPED, TurretState.HOLD_POSITION, FeederState.STOP, ClimberState.STOP);
    }
}
