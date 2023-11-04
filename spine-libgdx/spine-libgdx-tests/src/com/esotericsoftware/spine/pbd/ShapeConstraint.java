package com.esotericsoftware.spine.pbd;

public class ShapeConstraint extends BaseConstraint{

    int n_bones;
    int n_cons;
    DeformMesh deformMesh;
    LbsData lbsData;
    double alpha;
    double dt;

    public ShapeConstraint(DeformMesh deformMesh, LbsData lbsData, PhysicsSceneData sceneData,double alpha){
        this.deformMesh = deformMesh;
        this.lbsData = lbsData;
        n_bones = lbsData.n_bones;
        n_cons = lbsData.weightsIndex.length;
        dt = sceneData.dt;
        this.alpha = alpha / (dt * dt);
    }

    @Override
    public void init() {

    }

    @Override
    public void preUpdateProject() {

    }

    @Override
    public void project(){
        for(int j=0; j<n_bones; j++){
            project_single_bone(j);
        }
    }

    public void project_single_bone(int j){
        int startidx = lbsData.weightsStart[j];
        int endidx = startidx + lbsData.weightsCount[j];
        if (startidx == endidx) {
            return;
        }
        for (int idx = startidx; idx < endidx; idx++){
            int i = lbsData.weightsIndex[idx];
            double w = lbsData.weights[idx];
            Vec2 v_rig= ArrayOpr.getVec2(lbsData.getRigVerts(), i);
            Vec2 v = ArrayOpr.getVec2(deformMesh.vertices, i);
            v.sub(v_rig);
            // v.div(1.0 + alpha / w);
            ArrayOpr.addVec2(deformMesh.vertices, i, v, -w);
        }
    }









}
