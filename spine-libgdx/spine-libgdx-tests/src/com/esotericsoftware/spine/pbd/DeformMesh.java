package com.esotericsoftware.spine.pbd;

public class DeformMesh {

    int n_verts;
    int n_faces;

    double[] refVertices;
    double[] vertices;
    short[] indices;
    double[] faceMass;
    double[] vertMass;

    MeshData meshData;


    public DeformMesh(MeshData data){
        n_verts = data.getVertNum();
        n_faces = data.getFaceNum();
        vertices = data.getVertices();
        indices = data.getIndices();
        refVertices = data.getRefVertices();
        faceMass = new double[n_faces];
        vertMass = new double[n_verts];
        meshData = data;
        setMeshData();
    }

    public void setMeshData(){
        for(int i=0; i<n_faces; i++){
            Vec2 v0 = ArrayOpr.getVec2(vertices, indices[i*3], getScale());
            Vec2 v1 = ArrayOpr.getVec2(vertices, indices[i*3+1], getScale());
            Vec2 v2 = ArrayOpr.getVec2(vertices, indices[i*3+2], getScale());
            v0.sub(v2);
            v1.sub(v2);
            double area = Math.abs(v0.cross(v1))/2f;
            faceMass[i] = area;
            vertMass[indices[i*3]] += area/3;
            vertMass[indices[i*3+1]] += area/3;
            vertMass[indices[i*3+2]] += area/3;
        }
    }

    public double[] getVertices(){
        return vertices;
    }
    public double[] getRefVertices(){
        return refVertices;
    }
    public double getScale(){return meshData.getScale();}


}
