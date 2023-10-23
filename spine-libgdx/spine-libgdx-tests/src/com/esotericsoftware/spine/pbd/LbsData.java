package com.esotericsoftware.spine.pbd;

import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntArray;
import com.esotericsoftware.spine.Bone;
import com.esotericsoftware.spine.Slot;
import com.esotericsoftware.spine.attachments.MeshAttachment;

import java.util.ArrayList;

public class LbsData {
    MeshData meshData;
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
    Vec2[] bonePos;
    double[] rigVertices;
    float[] rigVerticesFloats;

    public LbsData(MeshData meshData, int[] bones, double[] vertices, Array<Bone> boneObjs, int vertexCount){

        n_verts = vertexCount;
        n_bones = boneObjs.size;
        rigVertices = new double[n_verts * 2];
        rigVerticesFloats = new float[n_verts * 2];

        this.meshData = meshData;
        this.boneObjs = new Bone[boneObjs.size];
        for(int i=0; i<boneObjs.size; i++){
            this.boneObjs[i] = boneObjs.get(i);
        }
        int n_bones = boneObjs.size;
        boneMats = new Mat2x2[boneObjs.size];
        bonePos = new Vec2[boneObjs.size];
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

        // print weights
        /* for(int i=0; i<n_bones;i++){
            System.out.println(weightsCount[i] + " " + weightsStart[i]);
            for(int j = weightsStart[i]; j<weightsStart[i]+weightsCount[i]; j++){
                System.out.println("    " + weightsIndex[j] + " " + weights[j]);
            }
        } */
    }

    // update bone states
    public void updateBoneStates(){
        for(int i=0; i<boneObjs.length; i++){
            Bone bone = boneObjs[i];
            boneMats[i] = new Mat2x2(bone.getA(), bone.getB(), bone.getC(), bone.getD());
            bonePos[i] = new Vec2(bone.getWorldX(), bone.getWorldY());
        }
    }

    public void updateLbsVerts(MeshAttachment meshAttachment, Slot slot, double scale){
        meshAttachment.computeWorldVertices(slot, 0, n_verts*2, rigVerticesFloats, 0, 2);
        for (int i=0; i<n_verts*2; i++){
            rigVertices[i] = rigVerticesFloats[i] / scale;
        }
    }

    public double[] getRigVerts(){
        return rigVertices;
    }

}
