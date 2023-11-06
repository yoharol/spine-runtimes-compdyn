package com.esotericsoftware.spine.pbd;


public final class ArrayOpr {

    private ArrayOpr() {
        throw new AssertionError("This class is a collect of array operations and cannot be instantiated.");
    }

    public static double[] convertArray(float[] sourceArray) {
        double[] targetArray = new double[sourceArray.length];
        for (int i = 0; i < sourceArray.length; i++) {
            targetArray[i] = sourceArray[i];
        }
        return targetArray;
    }

    public static float[] convertArray(double[] sourceArray) {
        float[] targetArray = new float[sourceArray.length];
        for (int i = 0; i < sourceArray.length; i++) {
            targetArray[i] = (float)sourceArray[i];
        }
        return targetArray;
    }



    public static void mulScale(double[] arr, int i, double scale){
        arr[i*2] *= scale;
        arr[i*2+1] *= scale;
    }

    public static Vec2 getVec2(double[] arr, int i){
        return new Vec2(arr[i*2], arr[i*2+1]);
    }

    public static Vec2 getVec2(double[] arr, int i, double scale){
        return new Vec2(arr[i*2]*scale, arr[i*2+1]*scale);
    }

    public static void setVec2(double[] arr, int i, Vec2 v){
        arr[i*2] = v.x();
        arr[i*2+1] = v.y();
    }

    public static void setVec2(double[] arr, int i, double x, double y){
        arr[i*2] = x;
        arr[i*2+1] = y;
    }


    public static void addVec2(double[] arr, int i, Vec2 v){
        arr[i*2] += v.x();
        arr[i*2+1] += v.y();
    }

    public static void addVec2(double[] arr, int i, Vec2 v, double scale){
        arr[i*2] += v.x()*scale;
        arr[i*2+1] += v.y()*scale;
    }

    public static void copyArr(double[] arr, int i, double[] arr2, int j){
        arr[i*2] = arr2[j*2];
        arr[i*2+1] = arr2[j*2+1];
    }

    public static void setAddArr(double[] arr, int i, double[] arr2, int j, double[] arr3, int k, double scale){
        arr[i*2] = arr2[j*2] + arr3[k*2]*scale;
        arr[i*2+1] = arr2[j*2+1] + arr3[k*2+1]*scale;
    }

    public static void addArr(double[] arr, int i, double[] arr2, int j, double scale){
        arr[i*2] += arr2[j*2]*scale;
        arr[i*2+1] += arr2[j*2+1]*scale;
    }

}
