package edu.wpi.first.wpilibj2.command;

import java.util.Set;

public class RepeatCommand implements Command {

    private final Command repeatCommand;

    private RepeatCommand(Command toRepeat) {
        this.repeatCommand = toRepeat;
        CommandGroupBase.requireUngrouped(toRepeat);
        CommandGroupBase.registerGroupedCommands(toRepeat);
    }

    public static Command sequence(Command... commands) {
        return CommandGroupBase.sequence(commands);
    }

    public static Command parallel(Command... commands) {
        return CommandGroupBase.parallel(commands);
    }

    @Override
    public void execute() {
        this.repeatCommand.execute();
        if (this.repeatCommand.isFinished()) {
            this.repeatCommand.end(false);
            this.repeatCommand.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.repeatCommand.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return repeatCommand.getRequirements();
    }
}
