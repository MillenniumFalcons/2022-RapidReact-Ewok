package team3647.frc2022.states;

public class IntakeState {

    public static final IntakeState DEPLOYED_INTAKE = new IntakeState(true);
    public static final IntakeState RETRACTED_INTAKE = new IntakeState(false);
    public static final IntakeState RETRACTED_STOPPED = new IntakeState(false);

    public IntakeState(boolean deployed) {
        this.deployed = deployed;
    }

    public final boolean deployed;
}
