package com.esotericsoftware.spine.pbd;

public class DeformConstraint extends BaseConstraint {

    DeformMesh deformMesh;
    int n_faces;
    double[] hydroLambda;
    double[] deviaLambda;

    double hydro_alpha;
    double devia_alpha;

    double dt;


    public DeformConstraint(DeformMesh deformMesh, PhysicsSceneData physicsSceneData, double hydro_alpha, double devia_alpha){
        this.deformMesh = deformMesh;
        n_faces = deformMesh.n_faces;
        hydroLambda = new double[n_faces];
        deviaLambda = new double[n_faces];
        this.hydro_alpha = hydro_alpha;
        this.devia_alpha = devia_alpha;
        dt = physicsSceneData.dt;
    }

    @Override
    public void init() {

    }

    @Override
    public void preUpdateProject() {
        for(int i=0; i<n_faces; i++){
            deviaLambda[i] = 0;
            hydroLambda[i] = 0;
        }
    }

    @Override
    public void project() {
        for(int k=0; k<n_faces; k++){
            int i0 = deformMesh.indices[k*3];
            int i1 = deformMesh.indices[k*3+1];
            int i2 = deformMesh.indices[k*3+2];
            Vec2 v0 = ArrayOpr.getVec2(deformMesh.vertices, i0);
            Vec2 v1 = ArrayOpr.getVec2(deformMesh.vertices, i1);
            Vec2 v2 = ArrayOpr.getVec2(deformMesh.vertices, i2);
            double w0 = 1.0f / deformMesh.vertMass[i0];
            double w1 = 1.0f / deformMesh.vertMass[i1];
            double w2 = 1.0f / deformMesh.vertMass[i2];
            v0.sub(v2);
            v1.sub(v2);
            Mat2x2 B = deformMesh.invB[k];
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
            double alpha_tilde_h = hydro_alpha / (dt * dt * deformMesh.faceMass[k]);

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
            double alpha_tilde_d = devia_alpha / (dt * dt * deformMesh.faceMass[k]);

            /*System.out.println("D" + new Mat2x2(v1, v2));
            System.out.println("B" + B);
            System.out.println("F" + F);
            System.out.println("C_H" + c_h);
            System.out.println("F det diff" + F.detDiff());
            System.out.println("par_ch" + par_ch);
            System.out.println(par_ch_x0);
            System.out.println(par_ch_x1);
            System.out.println(par_ch_x2);
            System.out.println("C_D" + c_d);
            System.out.println("par_cd" + par_cd);
            System.out.println(par_cd_x0);
            System.out.println(par_cd_x1);
            System.out.println(par_cd_x2);*/

            double sum_par_cdh = par_cd_x0.dot(par_ch_x0) * w0 + par_cd_x1.dot(par_ch_x1) * w1 + par_cd_x2.dot(par_ch_x2) * w2;
            double delta_lambda_h = (sum_par_cdh *
                    (c_d + alpha_tilde_d * deviaLambda[k]) -
                    (c_h + alpha_tilde_h * hydroLambda[k]) *
                            (alpha_tilde_d + sum_par_cd)) / (
                    (alpha_tilde_h + sum_par_ch) *
                            (alpha_tilde_d + sum_par_cd) -
                            sum_par_cdh * sum_par_cdh);
            double delta_lambda_d = - (c_d + alpha_tilde_d * deviaLambda[k] +
                    sum_par_cdh * delta_lambda_h) / (alpha_tilde_d + sum_par_cd);

            par_ch_x0.mul(delta_lambda_h*w0);
            ArrayOpr.addVec2(deformMesh.vertices, i0, par_ch_x0);
            par_ch_x1.mul(delta_lambda_h*w1);
            ArrayOpr.addVec2(deformMesh.vertices, i1, par_ch_x1);
            par_ch_x2.mul(delta_lambda_h*w2);
            ArrayOpr.addVec2(deformMesh.vertices, i2, par_ch_x2);

            par_cd_x0.mul(delta_lambda_d*w0);
            ArrayOpr.addVec2(deformMesh.vertices, i0, par_cd_x0);
            par_cd_x1.mul(delta_lambda_d*w1);
            ArrayOpr.addVec2(deformMesh.vertices, i1, par_cd_x1);
            par_cd_x2.mul(delta_lambda_d*w2);
            ArrayOpr.addVec2(deformMesh.vertices, i2, par_cd_x2);

            hydroLambda[k] += delta_lambda_h;
            deviaLambda[k] += delta_lambda_d;
        }
    }
}
