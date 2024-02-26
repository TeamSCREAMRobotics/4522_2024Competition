package com.team4522.lib.util;

import java.util.concurrent.atomic.AtomicBoolean;

public class RunOnce {
    
    private final AtomicBoolean hasRun = new AtomicBoolean(false);

    public void runOnce(Runnable runnable) {
        if (hasRun.compareAndSet(false, true)) {
            runnable.run();
        }
    }

    public void reset() {
        hasRun.set(false);
    }
}
