package com.team4522.lib.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ContinuousConditionalCommand{

    Command onTrue;
    Command onFalse;
    
    public ContinuousConditionalCommand(Command onTrue, Command onFalse){
        this.onTrue = onTrue;
        this.onFalse = onFalse;
    }

    public Supplier<Command> getCommand(BooleanSupplier condition){
        return condition.getAsBoolean() ? () -> onTrue : () -> onFalse;
    }
}