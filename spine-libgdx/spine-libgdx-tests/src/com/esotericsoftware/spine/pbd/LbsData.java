package com.esotericsoftware.spine.pbd;

import com.badlogic.gdx.utils.Array;
import com.esotericsoftware.spine.Bone;
import com.esotericsoftware.spine.Slot;
import com.esotericsoftware.spine.attachments.MeshAttachment;

public class LbsData {
    DeformMesh deformMesh;
    MeshAttachment meshAttachment;

    int n_verts;
    int n_bones;

    // Store weights in compressed format
    double[] weights;
    int[] weightsIndex;
    int[] weightsCount;
    int[] weightsStart;

    Bone[] boneObjs;

    Mat2x2[] boneMats;
    Vec2[] bonePos_ref;
    double[] rigVertices;
    float[] rigVerticesFloats;

    public LbsData(DeformMesh deformMesh, int[] bones, double[] vertices, Array<Bone> boneObjs, int vertexCount){

        n_verts = vertexCount;
        n_bones = boneObjs.size;
        rigVertices = new double[n_verts * 2];
        rigVerticesFloats = new float[n_verts * 2];

        this.deformMesh = deformMesh;
        this.boneObjs = new Bone[boneObjs.size];
        for(int i=0; i<boneObjs.size; i++){
            this.boneObjs[i] = boneObjs.get(i);
        }
        int n_bones = boneObjs.size;
        boneMats = new Mat2x2[boneObjs.size];
        bonePos_ref = new Vec2[boneObjs.size];
        updateBoneStates();

        for(int i=0; i<boneObjs.size; i++){
            this.boneObjs[i] = boneObjs.get(i);
        }

        weightsCount = new int[n_bones];
        weightsStart = new int[n_bones];
        int v_index = 0;
        int v_pointer = 0;
        for(int i=0; i<bones.length;){
            int n_w = bones[i];
            for(int j=0; j<n_w; j++){
                int b_idx = bones[i+j+1];
                weightsCount[b_idx] += 1;
                v_pointer+=3;
            }
            i=i+n_w+1;
            v_index += 1;
        }
        weightsStart[0] = 0;
        for(int i=1; i<n_bones; i++){
            weightsStart[i] = weightsStart[i-1] + weightsCount[i-1];
            System.out.println(weightsStart[i] + " " + weightsCount[i]);
        }
        int n_weights = weightsStart[n_bones-1] + weightsCount[n_bones-1];
        weightsIndex = new int[n_weights];
        weights = new double[n_weights];

        for(int i=0; i<n_bones; i++){
            weightsCount[i] = 0;
        }
        v_index = 0;
        v_pointer = 0;
        for(int i=0; i<bones.length;){
            int n_w = bones[i];
            for(int j=0; j<n_w; j++){
                int b_idx = bones[i+j+1];
                weightsIndex[weightsStart[b_idx] + weightsCount[b_idx]] = v_index;
                weights[weightsStart[b_idx] + weightsCount[b_idx]] = vertices[v_pointer+2];
                weightsCount[b_idx] += 1;
                v_pointer+=3;
            }
            i=i+n_w+1;
            v_index += 1;
        }

    }

    // update bone states
    public void updateBoneStates(){
        for(int i=0; i<boneObjs.length; i++){
            Bone bone = boneObjs[i];
            boneMats[i] = new Mat2x2(bone.getA(), bone.getB(), bone.getC(), bone.getD());
            bonePos_ref[i] = new Vec2(bone.getWorldX(), bone.getWorldY());
            bonePos_ref[i].div(deformMesh.getScale());
        }
    }

    public void updateLbsVerts(MeshAttachment meshAttachment, Slot slot, double scale){
        meshAttachment.computeWorldVertices(slot, 0, n_verts*2, rigVerticesFloats, 0, 2);
        for (int i=0; i<n_verts*2; i++){
            rigVertices[i] = rigVerticesFloats[i] / scale;
        }
    }

    public Bone getBone(int i){
        return boneObjs[i];
    }

    public double[] getRigVerts(){
        return rigVertices;
    }

    public Mat2x2 inverseMixed(int j, double blend){
        Mat2x2 P = new Mat2x2(0.0, 0.0, 0.0, 0.0);
        Mat2x2 Q = new Mat2x2(0.0, 0.0, 0.0, 0.0);
        int startidx = weightsStart[j];
        int endidx = startidx + weightsCount[j];
        for(int idx = startidx; idx < endidx; idx++){
            int i = weightsIndex[idx];
            double w = weights[idx];
            double x_bone = boneObjs[j].getWorldX() / deformMesh.getScale();
            double y_bone = boneObjs[j].getWorldY() / deformMesh.getScale();
            double x_bone_ref = bonePos_ref[j].x();
            double y_bone_ref = bonePos_ref[j].y();
            double x_ref = deformMesh.ref_vertices[i*2] - x_bone_ref;
            double y_ref = deformMesh.ref_vertices[i*2+1] - y_bone_ref;
            double x = deformMesh.vertices[i*2] - x_bone;
            double y = deformMesh.vertices[i*2+1] - y_bone;
            Mat2x2 D = new Mat2x2(x*x_ref, x*y_ref, y*x_ref, y*y_ref);
            D.mul(w*deformMesh.vertMass[i]);
            P.add(D);
            Mat2x2 B = new Mat2x2(x_ref*x_ref, x_ref*y_ref, y_ref*x_ref, y_ref*y_ref);
            B.mul(w*deformMesh.vertMass[i]);
            Q.add(B);
        }
        Q.inverse();
        P.dot(Q);
        Mat2x2[] RS = P.polarDecomposition();
        Q.set(RS[0]);
        Q.mul(blend);
        P.mul(1-blend);
        P.add(Q);
        P.dot(boneMats[j]);
        return P;
    }

    public int getN_bones(){
        return n_bones;
    }
}
