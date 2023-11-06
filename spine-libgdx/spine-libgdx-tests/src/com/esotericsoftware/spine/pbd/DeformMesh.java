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

    Mat2x2[] invB;

    public DeformMesh(MeshData data){
        n_verts = data.getVertNum();
        n_faces = data.getFaceNum();
        vertices = data.getVertices();
        indices = data.getIndices();
        refVertices = data.getRefVertices();
        faceMass = new double[n_faces];
        vertMass = new double[n_verts];
        invB = new Mat2x2[n_faces];
        for(int i=0; i<n_faces; i++){
            invB[i] = new Mat2x2(1, 0, 0, 1);
        }
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
            invB[i].copy(new Mat2x2(v0, v1));
            invB[i].inverse();
            double area = Math.abs(v1.cross(v2))/2f;
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
