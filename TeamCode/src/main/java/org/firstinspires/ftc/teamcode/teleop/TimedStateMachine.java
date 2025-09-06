package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class TimedStateMachine {
    public static class Step {
        public Runnable onStart = () -> {};
        public Runnable onLoop  = () -> {};
        public Runnable onStop  = () -> {};
        public double maxDurationMS = 0.0;            // 0 = no cap
        public BooleanSupplier doneWhen = () -> false; // true -> end early

        public Step onStart(Runnable r){ this.onStart = (r!=null?r:this.onStart); return this; }
        public Step onLoop (Runnable r){ this.onLoop  = (r!=null?r:this.onLoop ); return this; }
        public Step onStop (Runnable r){ this.onStop  = (r!=null?r:this.onStop ); return this; }
        public Step maxDurationMS(double s){ this.maxDurationMS = Math.max(0,s); return this; }
        public Step doneWhen(BooleanSupplier c){ this.doneWhen = (c!=null?c:this.doneWhen); return this; }
    }

    private final List<Step> steps = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private int idx = 0; private boolean running = false;

    public TimedStateMachine add(Step s){ steps.add(s); return this; }
    public TimedStateMachine clear(){ steps.clear(); idx=0; running=false; return this; }

    public void start(){
        if(steps.isEmpty()) return;
        idx = 0; running = true; timer.reset();
        steps.get(0).onStart.run();
    }
    public void cancel(){
        if(!running) return;
        steps.get(idx).onStop.run();
        running = false;
    }
    public void update(){
        if(!running) return;
        Step s = steps.get(idx);
        s.onLoop.run();
        boolean timeUp = s.maxDurationMS >0 && timer.milliseconds()>=s.maxDurationMS;
        if (s.doneWhen.getAsBoolean() || timeUp){
            s.onStop.run();
            idx++;
            if (idx>=steps.size()) running=false;
            else { timer.reset(); steps.get(idx).onStart.run(); }
        }
    }
    public boolean isRunning(){ return running; }
    public int currentIndex(){ return idx; }
    public int size(){ return steps.size(); }
    public double stateTime(){ return timer.seconds(); }
}
