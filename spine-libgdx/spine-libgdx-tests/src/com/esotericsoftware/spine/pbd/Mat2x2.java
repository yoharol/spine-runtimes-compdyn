package com.esotericsoftware.spine.pbd;

public class Mat2x2 {
    private final double[][] m = new double[2][2];

    // M = [a, b \\ c, d]
    public Mat2x2(double a, double b, double c, double d){
        m[0][0] = a;
        m[0][1] = b;
        m[1][0] = c;
        m[1][1] = d;
    }

    public double a(){
        return m[0][0];
    }

    public double b(){
        return m[0][1];
    }

    public double c(){
        return m[1][0];
    }

    public double d(){
        return m[1][1];
    }

    public Mat2x2(Mat2x2 _m){
        m[0][0] = _m.m[0][0];
        m[0][1] = _m.m[0][1];
        m[1][0] = _m.m[1][0];
        m[1][1] = _m.m[1][1];
    }

    public Mat2x2(Vec2 v1, Vec2 v2, boolean row){
        if (row){
            m[0][0] = v1.x();
            m[0][1] = v1.y();
            m[1][0] = v2.x();
            m[1][1] = v2.y();
        }
        else{
            m[0][0] = v1.x();
            m[0][1] = v2.x();
            m[1][0] = v1.y();
            m[1][1] = v2.y();
        }
    }

    public void copy(Mat2x2 _m){
        m[0][0] = _m.m[0][0];
        m[0][1] = _m.m[0][1];
        m[1][0] = _m.m[1][0];
        m[1][1] = _m.m[1][1];
    }

    // M = [col1, col2]
    public Mat2x2(Vec2 col1, Vec2 col2){
        m[0][0] = col1.x();
        m[0][1] = col2.x();
        m[1][0] = col1.y();
        m[1][1] = col2.y();
    }

    // inv(M)
    public void inverse(){
        double det = m[0][0]*m[1][1]-m[0][1]*m[1][0];
        double tmp = m[0][0];
        m[0][0] = m[1][1]/det;
        m[0][1] = -m[0][1]/det;
        m[1][0] = -m[1][0]/det;
        m[1][1] = tmp/det;
    }

    // d det(M) / d M
    public Mat2x2 detDiff(){
        return new Mat2x2(m[1][1], -m[1][0], -m[0][1], m[0][0]);
    }

    public double get(int i, int j){
        return m[i][j];
    }

    public void set(int i, int j, double v){
        m[i][j] = v;
    }

    public void set(double a, double b, double c, double d){
        m[0][0] = a;
        m[0][1] = b;
        m[1][0] = c;
        m[1][1] = d;
    }

    public void set(Mat2x2 _m){
        m[0][0] = _m.m[0][0];
        m[0][1] = _m.m[0][1];
        m[1][0] = _m.m[1][0];
        m[1][1] = _m.m[1][1];
    }

    public void add(Mat2x2 _m){
        m[0][0] += _m.m[0][0];
        m[0][1] += _m.m[0][1];
        m[1][0] += _m.m[1][0];
        m[1][1] += _m.m[1][1];
    }

    public void sub(Mat2x2 _m){
        m[0][0] -= _m.m[0][0];
        m[0][1] -= _m.m[0][1];
        m[1][0] -= _m.m[1][0];
        m[1][1] -= _m.m[1][1];
    }

    public void mul(double s){
        m[0][0] *= s;
        m[0][1] *= s;
        m[1][0] *= s;
        m[1][1] *= s;
    }

    public Vec2 dot(Vec2 v){
        return new Vec2(m[0][0]*v.x() + m[0][1]*v.y(), m[1][0]*v.x() + m[1][1]*v.y());
    }

    // M = M * _m
    public void dot(Mat2x2 _m){
        double m00= m[0][0]*_m.m[0][0] + m[0][1]*_m.m[1][0];
        double m01= m[0][0]*_m.m[0][1] + m[0][1]*_m.m[1][1];
        double m10= m[1][0]*_m.m[0][0] + m[1][1]*_m.m[1][0];
        double m11= m[1][0]*_m.m[0][1] + m[1][1]*_m.m[1][1];
        m[0][0] = m00;
        m[0][1] = m01;
        m[1][0] = m10;
        m[1][1] = m11;
    }

    // M = M * _m^T
    public void dotT(Mat2x2 _m){
        double m00 = m[0][0]*_m.m[0][0] + m[0][1]*_m.m[0][1];
        double m01 = m[0][0]*_m.m[1][0] + m[0][1]*_m.m[1][1];
        double m10 = m[1][0]*_m.m[0][0] + m[1][1]*_m.m[0][1];
        double m11 = m[1][0]*_m.m[1][0] + m[1][1]*_m.m[1][1];
        m[0][0] = m00;
        m[0][1] = m01;
        m[1][0] = m10;
        m[1][1] = m11;
    }

    public double ddot(Mat2x2 _m){
        return m[0][0]*_m.m[0][0]+m[0][1]*_m.m[0][1]+m[1][0]*_m.m[1][0]+m[1][1]*_m.m[1][1];
    }

    public double norm(){
        return (double)Math.sqrt(m[0][0]*m[0][0]+m[0][1]*m[0][1]+m[1][0]*m[1][0]+m[1][1]*m[1][1]);
    }

    public double norm_sqr(){
        return m[0][0]*m[0][0]+m[0][1]*m[0][1]+m[1][0]*m[1][0]+m[1][1]*m[1][1];
    }

    public static Mat2x2 identity(){
        return new Mat2x2(1, 0, 0, 1);
    }

    public static Mat2x2 zero(){
        return new Mat2x2(0, 0, 0, 0);
    }

    public double det(){
        return m[0][0]*m[1][1]-m[0][1]*m[1][0];
    }

    public Mat2x2 transpose(){
        return new Mat2x2(m[0][0], m[1][0], m[0][1], m[1][1]);
    }

    public Vec2 col(int c){
        return new Vec2(m[0][c], m[1][c]);
    }

    @Override
    public String toString(){
        return m[0][0] + " " + m[0][1] + "\n" + m[1][0] + " " + m[1][1] + "\n";
    }

    // fast polar decomposition of 2x2 matrix
    // https://ieeexplore.ieee.org/document/486688
    // return [R, S]
    // R: rotation part
    // S: symmetric part
    public Mat2x2[] polarDecomposition(){
        Mat2x2[] RS = new Mat2x2[2];
        double a = m[0][0];
        double b = m[0][1];
        double c = m[1][0];
        double d = m[1][1];
        double e = (a+d)/2;
        double f = (a-d)/2;
        double h = (b-c)/2;
        double g = (b+c)/2;
        double q = Math.sqrt(e*e+h*h);
        double r = Math.sqrt(f*f+g*g);
        double w1 = q+r;
        double w2 = q-r;
        double t1 = Math.atan2(g, f);
        double t2 = Math.atan2(h, e);
        double gamma = (t2+t1)/2;
        double beta = (t2-t1)/2;
        // svd decomposition
        Mat2x2 U = new Mat2x2(Math.cos(beta), Math.sin(beta), -Math.sin(beta), Math.cos(beta));
        Mat2x2 V = new Mat2x2(Math.cos(gamma), Math.sin(gamma), -Math.sin(gamma), Math.cos(gamma));
        Mat2x2 S = new Mat2x2(w1, 0, 0, w2);
        // rotation part of polar decomposition
        RS[0] = new Mat2x2(U);
        RS[0].dot(V);
        // symmetric part of polar decomposition
        RS[1] = new Mat2x2(V);
        RS[1].set(V.transpose());
        RS[1].dot(S);
        RS[1].dot(V);
        return RS;
    }



}
