package com.esotericsoftware.spine.pbd;

public class Vec2 {
    private final double[] v = new double[2];

    public Vec2(double x, double y){
        this.v[0] = x;
        this.v[1] = y;
    }

    public Vec2(Vec2 _v){
        v[0] = _v.v[0];
        v[1] = _v.v[1];
    }

    public double x(){
        return v[0];
    }

    public double y(){
        return v[1];
    }

    public void setX(double x){
        v[0] = x;
    }

    public void setY(double y){
        v[1] = y;
    }

    public void set(double x, double y){
        v[0] = x;
        v[1] = y;
    }

    public void set(Vec2 _v){
        v[0] = _v.x();
        v[1] = _v.y();
    }

    public void add(Vec2 _v){
        v[0] += _v.x();
        v[1] += _v.y();
    }

    public void add(double x, double y){
        v[0] += x;
        v[1] += y;
    }

    public void sub(Vec2 _v){
        v[0] -= _v.x();
        v[1] -= _v.y();
    }

    public void sub(double x, double y){
        v[0] -= x;
        v[1] -= y;
    }

    public void mul(double s){
        v[0] *= s;
        v[1] *= s;
    }

    public void div(double s){
        v[0] /= s;
        v[1] /= s;
    }

    public double dot(Vec2 _v){
        return v[0] * _v.x() + v[1] * _v.y();
    }

    public double dot(double x, double y){
        return v[0] * x + v[1] * y;
    }

    public double norm(){
        return (double)Math.sqrt(v[0] * v[0] + v[1] * v[1]);
    }

    public double norm_sqr(){
        return v[0] * v[0] + v[1] * v[1];
    }

    public double norm_dist(Vec2 _v){
        return (double)Math.sqrt((_v.x() - v[0]) * (_v.x() - v[0]) + (_v.y() - v[1]) * (_v.y() - v[1]));
    }

    public double norm_dist_sqr(Vec2 _v){
        return (_v.x() - v[0]) * (_v.x() - v[0]) + (_v.y() - v[1]) * (_v.y() - v[1]);
    }

    public void normalize(){
        double n = norm();
        v[0] /= n;
        v[1] /= n;
    }

    public void normalize_dist(Vec2 _v){
        v[0] -= _v.v[0];
        v[1] -= _v.v[1];
        double n = norm();
        v[0] /= n;
        v[1] /= n;
    }

    public Vec2 normalized(){
        Vec2 _v = new Vec2(this);
        _v.normalize();
        return _v;
    }

    public double cross(Vec2 _v){
        return v[0] * _v.y() - v[1] * _v.x();
    }

    @Override
    public String toString(){
        return "(" + v[0] + ", " + v[1] + ")";
    }

    public static Vec2 zero(){
        return new Vec2(0, 0);
    }

    public static Vec2 right(){
        return new Vec2(1, 0);
    }

    public static Vec2 up(){
        return new Vec2(0, 1);
    }

}
