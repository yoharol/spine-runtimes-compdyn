package com.esotericsoftware.spine.pbd;

public class MeshData {

    int n_verts;
    int n_faces;

    double[] vertices;
    short[] indices;

    public MeshData(double[] vertices, short[] indices){
        this.vertices = vertices;
        this.indices = indices;
        n_verts = vertices.length/2;
        n_faces = indices.length/3;
    }

    public double[] getVertices(){
        return vertices;
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

}
