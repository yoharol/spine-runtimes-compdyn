package com.esotericsoftware.spine.pbd;

import com.esotericsoftware.spine.attachments.MeshAttachment;

public class LbsConstraint extends  BaseConstraint{

    int n_bones;

    DeformMesh deformMesh;
    LbsData lbsData;
    double[][] lambda;
    double[][] c;
    double[][] delta_lambda;
    double[][] c_deriv;
    double alpha;
    double dt;

    public LbsConstraint(DeformMesh deformMesh, LbsData lbsData, PhysicsSceneData sceneData,double alpha){
        this.deformMesh = deformMesh;
        this.lbsData = lbsData;
        n_bones = lbsData.n_bones;
        dt = sceneData.dt;
        this.alpha = alpha / (dt * dt);
        lambda = new double[n_bones][6];
        c = new double[n_bones][6];
        delta_lambda = new double[n_bones][6];
        c_deriv = new double[n_bones][3];
    }

    @Override
    public void init() {
        for (int j=0; j<n_bones; j++){
            int startidx = lbsData.weightsStart[j];
            int endidx = startidx + lbsData.weightsCount[j];
            for(int k=startidx; k<endidx; k++){
                int i=lbsData.weightsIndex[k];
                double w = lbsData.weights[k];
                double m = deformMesh.vertMass[i];
                Vec2 v_ref= ArrayOpr.getVec2(deformMesh.ref_vertices, i);
                c_deriv[j][0] += m * w * w * v_ref.x() * v_ref.x();
                c_deriv[j][1] += m * w * w * v_ref.y() * v_ref.y();
                c_deriv[j][2] += m * w * w;
            }
        }
    }

    @Override
    public void preUpdateProject() {
        for(int j=0; j<n_bones; j++){
            for(int k=0; k<6; k++){
                lambda[j][k] = 0;
            }
        }
    }

    @Override
    public void project(){
        for(int j=0; j<n_bones; j++){
            for (int k = 0; k < 6; k++) {
                c[j][k] = 0;
            }

            int startidx = lbsData.weightsStart[j];
            int endidx = startidx + lbsData.weightsCount[j];
            if (startidx == endidx){
                continue;
            }

            for(int idx = startidx; idx < endidx; idx ++){
                int i = lbsData.weightsIndex[idx];
                double w = lbsData.weights[idx];
                double m = deformMesh.vertMass[i];
                Vec2 vc = ArrayOpr.getVec2(deformMesh.vertices, i);
                vc.sub(ArrayOpr.getVec2(lbsData.rigVertices, i));
                Vec2 v_ref = ArrayOpr.getVec2(deformMesh.ref_vertices, i);
                c[j][0] += m * w * v_ref.x() * vc.x();
                c[j][1] += m * w * v_ref.x() * vc.y();
                c[j][2] += m * w * v_ref.y() * vc.x();
                c[j][3] += m * w * v_ref.y() * vc.y();
                c[j][4] += m * w * vc.x();
                c[j][5] += m * w * vc.y();
            }

            for(int k=0; k<6; k++){
                int kdx = k/2;
                delta_lambda[j][k] = - (c[j][k] + alpha * lambda[j][k]) / (
                        c_deriv[j][kdx] + alpha);
                lambda[j][k] += delta_lambda[j][k];
            }

            for(int idx = startidx; idx < endidx; idx ++){
                int i = lbsData.weightsIndex[idx];
                double w = lbsData.weights[idx];
                Vec2 v_delta = new Vec2(0.0, 0.0);
                Vec2 v_ref = ArrayOpr.getVec2(deformMesh.ref_vertices, i);
                v_delta.add(w * delta_lambda[j][0] * v_ref.x(), w * delta_lambda[j][1] * v_ref.x());
                v_delta.add(w * delta_lambda[j][2] * v_ref.y(), w * delta_lambda[j][3] * v_ref.y());
                v_delta.add(w * delta_lambda[j][4], w * delta_lambda[j][5]);
                ArrayOpr.addVec2(deformMesh.vertices, i, v_delta);
            }
        }
    }





}
