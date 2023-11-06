package com.esotericsoftware.spine.pbd;

public class DeformConstraint extends BaseConstraint {

    DeformMesh deformMesh;
    int n_faces;

    double hydro_alpha;
    double devia_alpha;

    double dt;


    public DeformConstraint(DeformMesh deformMesh, PhysicsSceneData physicsSceneData, double hydro_alpha, double devia_alpha) {
        this.deformMesh = deformMesh;
        n_faces = deformMesh.n_faces;
        dt = physicsSceneData.dt;
        this.hydro_alpha = hydro_alpha / (dt * dt);
        this.devia_alpha = devia_alpha / (dt * dt);
    }

    @Override
    public void init() {

    }

    @Override
    public void preUpdateProject() {
    }

    @Override
    public void project() {
        for(int k=0; k<n_faces; k++){
            int i0 = deformMesh.indices[k*3];
            int i1 = deformMesh.indices[k*3+1];
            int i2 = deformMesh.indices[k*3+2];
            Vec2 v0 = ArrayOpr.getVec2(deformMesh.vertices, i0, deformMesh.getScale());
            Vec2 v1 = ArrayOpr.getVec2(deformMesh.vertices, i1, deformMesh.getScale());
            Vec2 v2 = ArrayOpr.getVec2(deformMesh.vertices, i2, deformMesh.getScale());
            Vec2 v0_ref = ArrayOpr.getVec2(deformMesh.refVertices, i0, deformMesh.getScale());
            Vec2 v1_ref = ArrayOpr.getVec2(deformMesh.refVertices, i1, deformMesh.getScale());
            Vec2 v2_ref = ArrayOpr.getVec2(deformMesh.refVertices, i2, deformMesh.getScale());
            double w0 = 1.0 / deformMesh.vertMass[i0];
            double w1 = 1.0 / deformMesh.vertMass[i1];
            double w2 = 1.0 / deformMesh.vertMass[i2];
            if(w0 + w1 + w2 == 0){
                continue;
            }
            v0.sub(v2);
            v1.sub(v2);
            v0_ref.sub(v2_ref);
            v1_ref.sub(v2_ref);
            Mat2x2 B = new Mat2x2(v0_ref, v1_ref);
            B.inverse();
            Mat2x2 F = new Mat2x2(v0, v1);
            F.dot(B);

            double c_h = F.det() - 1f;
            Mat2x2 par_ch = F.detDiff();
            par_ch.dotT(B);
            Vec2 par_ch_x0 = par_ch.col(0);
            Vec2 par_ch_x1 = par_ch.col(1);
            Vec2 par_ch_x2 = new Vec2(par_ch_x0);
            par_ch_x2.add(par_ch_x1);
            par_ch_x2.mul(-1f);
            double sum_par_ch =  par_ch_x0.norm_sqr() * w0 + par_ch_x1.norm_sqr() * w1 + par_ch_x2.norm_sqr() * w2;
            double alpha_tilde_h = hydro_alpha / (deformMesh.faceMass[k]);

            double c_d = F.ddot(F) - 2f;
            Mat2x2 par_cd = new Mat2x2(F);
            par_cd.mul(2f);
            par_cd.dotT(B);
            Vec2 par_cd_x0 = par_cd.col(0);
            Vec2 par_cd_x1 = par_cd.col(1);
            Vec2 par_cd_x2 = new Vec2(par_cd_x0);
            par_cd_x2.add(par_cd_x1);
            par_cd_x2.mul(-1f);
            double sum_par_cd = par_cd_x0.norm_sqr() * w0 + par_cd_x1.norm_sqr() * w1 + par_cd_x2.norm_sqr() * w2;
            double alpha_tilde_d = devia_alpha / (deformMesh.faceMass[k]);

            double sum_par_cdh = par_cd_x0.dot(par_ch_x0) * w0 + par_cd_x1.dot(par_ch_x1) * w1 + par_cd_x2.dot(par_ch_x2) * w2;
            double delta_lambda_h = (sum_par_cdh *
                    (c_d) -
                    (c_h) *
                            (alpha_tilde_d + sum_par_cd)) / (
                    (alpha_tilde_h + sum_par_ch) *
                            (alpha_tilde_d + sum_par_cd) -
                            sum_par_cdh * sum_par_cdh);
            double delta_lambda_d = - (c_d +
                    sum_par_cdh * delta_lambda_h) / (alpha_tilde_d + sum_par_cd);

            par_ch_x0.mul(delta_lambda_h*w0);
            ArrayOpr.addVec2(deformMesh.vertices, i0, par_ch_x0, 1.0 / deformMesh.getScale());
            par_ch_x1.mul(delta_lambda_h*w1);
            ArrayOpr.addVec2(deformMesh.vertices, i1, par_ch_x1, 1.0 / deformMesh.getScale());
            par_ch_x2.mul(delta_lambda_h*w2);
            ArrayOpr.addVec2(deformMesh.vertices, i2, par_ch_x2, 1.0 / deformMesh.getScale());

            par_cd_x0.mul(delta_lambda_d*w0);
            ArrayOpr.addVec2(deformMesh.vertices, i0, par_cd_x0, 1.0 / deformMesh.getScale());
            par_cd_x1.mul(delta_lambda_d*w1);
            ArrayOpr.addVec2(deformMesh.vertices, i1, par_cd_x1, 1.0 / deformMesh.getScale());
            par_cd_x2.mul(delta_lambda_d*w2);
            ArrayOpr.addVec2(deformMesh.vertices, i2, par_cd_x2, 1.0 / deformMesh.getScale());
        }
    }
}
