package com.esotericsoftware.spine.pbd;

public class PhysicsSceneData {
    public Vec2 gravity = new Vec2(0, -10);
    public double dt = 1.0 / (60 * 15);
    public int iterations = 15;
    public double damping = 1f;

    public void setGravity(double x, double y){
        gravity.set(x, y);
    }

    public void setFps(int fps, int iterations){
        this.iterations = iterations;
        dt = 1.0f / (fps * iterations);
    }

    public void setDamping(double damping){
        this.damping = damping;
    }


}
