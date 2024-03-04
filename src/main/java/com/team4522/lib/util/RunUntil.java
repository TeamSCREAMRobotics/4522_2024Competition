package com.team4522.lib.util;

import java.util.concurrent.atomic.AtomicBoolean;

public class RunUntil {
    private final AtomicBoolean shouldStop = new AtomicBoolean(false);

    public void runUntil(Runnable runnable, boolean stopCondition) {
        if (!shouldStop.get() && stopCondition) {
            runnable.run();
            shouldStop.compareAndSet(false, true);
        }
    }

    public void reset() {
        shouldStop.set(false);
    }

    public boolean hasStopped(){
        return shouldStop.get();
    }
}

