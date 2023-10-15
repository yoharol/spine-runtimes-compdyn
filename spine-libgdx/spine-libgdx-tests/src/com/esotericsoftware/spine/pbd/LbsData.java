package com.esotericsoftware.spine.pbd;

import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntArray;
import com.esotericsoftware.spine.Bone;

import java.util.ArrayList;

public class LbsData {
    MeshData meshData;

    int n_verts;

    // Store weights in compressed format
    double[] weights;
    int[] weightsIndex;
    int[] weightsCount;
    int[] weightsStart;

    Bone[] boneObjs;

    Mat2x2[] boneMats;
    Vec2[] bonePos;

    public LbsData(MeshData meshData, int[] bones, double[] vertices, Array<Bone> boneObjs, int vertexCount){
        this.meshData = meshData;
        this.boneObjs = new Bone[boneObjs.size];
        for(int i=0; i<boneObjs.size; i++){
            this.boneObjs[i] = boneObjs.get(i);
        }
        boneMats = new Mat2x2[boneObjs.size];
        bonePos = new Vec2[boneObjs.size];
        updateBoneStates();

        for(int i=0; i<boneObjs.size; i++){
            this.boneObjs[i] = boneObjs.get(i);
        }

        n_verts = vertexCount;

        ArrayList<Double> w = new ArrayList<Double>();
        IntArray wIndex = new IntArray();
        weightsCount = new int[vertexCount];
        weightsStart = new int[vertexCount];

        int v_index = 0;
        int v_pointer = 0;
        for(int i=0; i<bones.length;){
            int n_weights = bones[i];
            weightsCount[v_index] = n_weights;
            weightsStart[v_index] = w.size();
            for(int j=0; j<n_weights; j++){
                int b_idx = bones[i+j+1];
                wIndex.add(b_idx);
                double weight = vertices[v_pointer+2];
                w.add(weight);
                v_pointer+=3;
            }
            i=i+n_weights+1;
            v_index += 1;
        }

        System.out.println(n_verts + " " + v_index);

        weights = new double[w.size()];
        for(int i=0; i<w.size(); i++){
            weights[i] = w.get(i);
        }
        weightsIndex = wIndex.items;

        // print weights
        /*for(int i=0; i<vertexCount;i++){
            System.out.println(weightsCount[i] + " " + weightsStart[i]);
            for(int j = weightsStart[i]; j<weightsStart[i]+weightsCount[i]; j++){
                System.out.println("    " + weightsIndex[j] + " " + weights[j]);
            }
        }*/
    }

    // update bone states
    public void updateBoneStates(){
        for(int i=0; i<boneObjs.length; i++){
            Bone bone = boneObjs[i];
            boneMats[i] = new Mat2x2(bone.getA(), bone.getB(), bone.getC(), bone.getD());
            bonePos[i] = new Vec2(bone.getWorldX(), bone.getWorldY());
        }
    }

}
