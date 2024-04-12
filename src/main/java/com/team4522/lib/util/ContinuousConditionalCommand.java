package com.team4522.lib.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ContinuousConditionalCommand extends Command{
    final Command m_whileTrue;
    final Command m_whileFalse;

    final BooleanSupplier m_condition;

    Command m_selectedCommand;
    Command m_lastCommand;

    public ContinuousConditionalCommand(Command whileTrue, Command whileFalse, BooleanSupplier condition){
        m_whileTrue = requireNonNullParam(whileTrue, "whileTrue", "ConditionalCommand");
        m_whileFalse = requireNonNullParam(whileFalse, "whileFalse", "ConditionalCommand");
        m_condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(m_whileTrue, m_whileFalse);

        m_requirements.addAll(m_whileTrue.getRequirements());
        m_requirements.addAll(m_whileFalse.getRequirements());
    }

    @Override
    public void initialize() {
        if(m_condition.getAsBoolean()){
            m_selectedCommand = m_whileTrue;
        } else {
            m_selectedCommand = m_whileFalse;
        }
        m_selectedCommand.initialize();
    }

    @Override
    public void execute() {
        if(m_condition.getAsBoolean()){
            m_selectedCommand = m_whileTrue;
        } else {
            m_selectedCommand = m_whileFalse;
        }

        if(m_selectedCommand != m_lastCommand){
            m_selectedCommand.initialize();
        }

        m_selectedCommand.execute();
        m_lastCommand = m_selectedCommand;
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }
}
