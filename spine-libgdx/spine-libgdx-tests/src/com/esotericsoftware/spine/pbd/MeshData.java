package com.esotericsoftware.spine.pbd;

public class MeshData {

    int n_verts;
    int n_faces;

    double[] vertices;
    double[] ref_vertices;
    short[] indices;

    double scale;

    public MeshData(double[] vertices, short[] indices){
        this.vertices = vertices;
        this.indices = indices;
        n_verts = vertices.length/2;
        n_faces = indices.length/3;

        double min_x = Double.MAX_VALUE;
        double min_y = Double.MAX_VALUE;
        double max_x = Double.MIN_VALUE;
        double max_y = Double.MIN_VALUE;
        for(int i=0; i<n_verts; i++){
            min_x = Math.min(min_x, vertices[i*2]);
            min_y = Math.min(min_y, vertices[i*2+1]);
            max_x = Math.max(max_x, vertices[i*2]);
            max_y = Math.max(max_y, vertices[i*2+1]);
        }
        scale = Math.max(max_x - min_x, max_y - min_y);
        for(int i=0; i<n_verts*2;i++){
            vertices[i] /= scale;
        }
        ref_vertices = new double[vertices.length];
        System.arraycopy(vertices, 0, ref_vertices, 0, vertices.length);
    }

    public double[] getVertices(){
        return vertices;
    }

    public double[] getRefVertices(){
        return ref_vertices;
    }

    public short[] getIndices(){
        return indices;
    }

    public int getVertNum(){
        return n_verts;
    }

    public int getFaceNum(){
        return n_faces;
    }

    public double getScale(){return scale;}

}
