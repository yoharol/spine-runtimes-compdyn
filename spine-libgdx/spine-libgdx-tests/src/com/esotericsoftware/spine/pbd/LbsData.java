package com.esotericsoftware.spine.pbd;

import com.badlogic.gdx.utils.Array;
import com.esotericsoftware.spine.Bone;
import com.esotericsoftware.spine.Slot;
import com.esotericsoftware.spine.attachments.MeshAttachment;

public class LbsData {
    DeformMesh deformMesh;

    int n_verts;
    int n_bones;

    // Store weights in compressed format
    double[] weights;
    int[] weightsIndex;
    int[] weightsCount;
    int[] weightsStart;

    Bone[] boneObjs;

    Mat2x2[] boneMatsRef;
    Vec2[] bonePosRef;
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
        boneMatsRef = new Mat2x2[boneObjs.size];
        bonePosRef = new Vec2[boneObjs.size];
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

        for(int i=0; i<n_bones; i++){
            // print bone data and weights data
            System.out.println("[pbd.LbsData.LbsData] " + "Bone " + i + ": " + boneObjs.get(i).getData().getName());
            System.out.println("[pbd.LbsData.LbsData] " + weightsStart[i] + " " + weightsCount[i]);
        }

    }

    // update bone states
    public void updateBoneStates(){
        for(int i=0; i<boneObjs.length; i++){
            Bone bone = boneObjs[i];
            boneMatsRef[i] = new Mat2x2(bone.getA(), bone.getB(), bone.getC(), bone.getD());
            bonePosRef[i] = new Vec2(bone.getWorldX(), bone.getWorldY());
        }
    }

    public void updateLbsVerts(MeshAttachment meshAttachment, Slot slot){
        meshAttachment.computeWorldVertices(slot, 0, n_verts*2, rigVerticesFloats, 0, 2);
        for (int i=0; i<n_verts*2; i++){
            rigVertices[i] = rigVerticesFloats[i];
        }
    }

    public int getStartIndex(int j){
        return weightsStart[j];
    }

    public int getEndIndex(int j){
        return weightsStart[j] + weightsCount[j];
    }

    public Bone getBone(int i){
        return boneObjs[i];
    }

    public double[] getRigVerts(){
        return rigVertices;
    }

    // extract bone transformation from vertices
    // blend: 0.0 - 1.0, blending of affine transformation and rotation
    //      0.0: only affine transformation
    //      1.0: only rotation
    public Mat2x2 inverseMixed(int j, double blend){
        Mat2x2 P = new Mat2x2(0.0, 0.0, 0.0, 0.0);
        Mat2x2 Q = new Mat2x2(0.0, 0.0, 0.0, 0.0);
        int startidx = weightsStart[j];
        int endidx = startidx + weightsCount[j];
        for(int idx = startidx; idx < endidx; idx++){
            int i = weightsIndex[idx];
            double w = weights[idx];
            double x_bone = boneObjs[j].getWorldX();
            double y_bone = boneObjs[j].getWorldY();
            double x_bone_ref = bonePosRef[j].x();
            double y_bone_ref = bonePosRef[j].y();
            double x_ref = deformMesh.refVertices[i*2] - x_bone_ref;
            double y_ref = deformMesh.refVertices[i*2+1] - y_bone_ref;
            double x = deformMesh.vertices[i*2] - x_bone;
            double y = deformMesh.vertices[i*2+1] - y_bone;
            x *= deformMesh.getScale();
            y *= deformMesh.getScale();
            x_ref *= deformMesh.getScale();
            y_ref *= deformMesh.getScale();
            Mat2x2 D = new Mat2x2(x*x_ref, x*y_ref, y*x_ref, y*y_ref);
            D.mul(w*deformMesh.vertMass[i]);
            P.add(D);
            Mat2x2 B = new Mat2x2(x_ref*x_ref, x_ref*y_ref, y_ref*x_ref, y_ref*y_ref);
            B.mul(w*deformMesh.vertMass[i]);
            Q.add(B);
        }
        Q.inverse();
        P.dot(Q);
        if (blend > 0.0) {
            Mat2x2[] RS = P.polarDecomposition();
            Q.set(RS[0]);
            Q.mul(blend);
            P.mul(1 - blend);
            P.add(Q);
        }
        P.dot(boneMatsRef[j]);
        return P;
    }

    public int getN_bones(){
        return n_bones;
    }


    public int[] getFixedBones(int[] free_bones){
        int[] fixed_bones = new int[n_bones - free_bones.length];
        int idx = 0;
        for(int i=0; i<n_bones; i++){
            boolean is_free = false;
            for(int j=0; j<free_bones.length; j++){
                if(i == free_bones[j]){
                    is_free = true;
                    break;
                }
            }
            if(!is_free){
                fixed_bones[idx] = i;
                idx += 1;
            }
        }
        return fixed_bones;
    }

}
