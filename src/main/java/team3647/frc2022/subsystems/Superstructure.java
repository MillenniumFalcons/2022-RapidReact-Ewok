package team3647.frc2022.subsystems;

import team3647.frc2022.states.RobotState;

public class Superstructure {
    private final PivotClimber m_climber;
    private final ColumnBottom m_columnBottom;
    private final VerticalRollers m_verticalRollers;
    private final ColumnTop m_column_top;
    private final Intake m_intake;
    private final Turret m_turret;
    private final Flywheel m_flywheel;

    private RobotState currentState, aimedState;

    public Superstructure(
            PivotClimber m_climber,
            ColumnBottom m_columnBottom,
            VerticalRollers m_verticalRollers,
            ColumnTop m_column_top,
            Intake m_intake,
            Turret m_turret,
            Flywheel m_flywheel) {
        this.m_climber = m_climber;
        this.m_columnBottom = m_columnBottom;
        this.m_verticalRollers = m_verticalRollers;
        this.m_column_top = m_column_top;
        this.m_intake = m_intake;
        this.m_turret = m_turret;
        this.m_flywheel = m_flywheel;
    }

    public void setState(RobotState state) {
        this.aimedState = state;
    }

    public void periodic(double timestamp) {}

    public boolean isClimbing() {
        return RobotState.CLIMB.equals(aimedState);
    }
}
