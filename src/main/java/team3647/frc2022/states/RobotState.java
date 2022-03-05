package team3647.frc2022.states;

public class RobotState {
    public static final RobotState STOPPED = new RobotState();
    public static final RobotState CLIMB =
            new RobotState(
                    ShooterState.STOPPED,
                    TurretState.CLIMB,
                    FeederState.STOP,
                    ClimberState.CLIMB,
                    IntakeState.RETRACTED_STOPPED);

    public static final RobotState IDLE =
            new RobotState(
                    ShooterState.IDLE,
                    TurretState.HOLD_POSITION,
                    FeederState.STOP,
                    ClimberState.STOP,
                    IntakeState.RETRACTED_STOPPED);

    public static final RobotState INTAKE_IDLE =
            new RobotState(
                    ShooterState.IDLE,
                    TurretState.HOLD_POSITION,
                    FeederState.INTAKE,
                    ClimberState.STOP,
                    IntakeState.DEPLOYED_INTAKE);

    public static final RobotState INTAKE_SHOOT =
            new RobotState(
                    ShooterState.SHOOT,
                    TurretState.AIM,
                    FeederState.SHOOT,
                    ClimberState.STOP,
                    IntakeState.DEPLOYED_INTAKE);

    public static final RobotState IDLE_SHOOT =
            new RobotState(
                    ShooterState.SHOOT,
                    TurretState.AIM,
                    FeederState.SHOOT,
                    ClimberState.STOP,
                    IntakeState.RETRACTED_STOPPED);

    public ShooterState shooterState;
    public TurretState turretState;
    public FeederState feederState;
    public ClimberState climberState;
    public IntakeState intakeState;

    public RobotState(
            ShooterState shooterState,
            TurretState turretState,
            FeederState feederState,
            ClimberState climberState,
            IntakeState intakeState) {
        this.shooterState = shooterState;
        this.turretState = turretState;
        this.feederState = feederState;
        this.climberState = climberState;
        this.intakeState = intakeState;
    }

    public RobotState() {
        this(
                ShooterState.STOPPED,
                TurretState.HOLD_POSITION,
                FeederState.STOP,
                ClimberState.STOP,
                IntakeState.RETRACTED_STOPPED);
    }
}
