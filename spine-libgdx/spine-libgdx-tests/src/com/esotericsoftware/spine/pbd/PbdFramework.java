package com.esotericsoftware.spine.pbd;

import com.badlogic.gdx.utils.IntArray;

import java.util.ArrayList;

public class PbdFramework {
    ArrayList<BaseConstraint> constraints = new ArrayList<BaseConstraint>();
    IntArray constraintLevels = new IntArray();
    int n_verts;
    double[] vertices;
    double[] vertices_cache;
    double[] ref_vertices;
    double[] velocities;
    PhysicsSceneData sceneData;

    public PbdFramework(PhysicsSceneData sceneData, DeformMesh deformMesh){
        this.sceneData = sceneData;
        vertices = deformMesh.vertices;
        ref_vertices = deformMesh.ref_vertices;
        velocities = new double[vertices.length];
        vertices_cache = new double[vertices.length];
        n_verts = deformMesh.n_verts;
    }

    public void addConstraint(BaseConstraint constraint, int level){
        constraints.add(constraint);
        constraintLevels.add(level);
    }

    public void addConstraint(BaseConstraint constraint){
        constraints.add(constraint);
        constraintLevels.add(0);
    }

    public void makePrediction(){
        for(int i=0; i<n_verts; i++){
            ArrayOpr.copyArr(vertices_cache, i, vertices, i);
            ArrayOpr.addArr(vertices, i, velocities, i, sceneData.dt);
            ArrayOpr.addVec2(vertices, i, sceneData.gravity, sceneData.dt*sceneData.dt);
        }
    }

    public void updateVelocity(){
        for(int i=0; i<n_verts; i++){
            ArrayOpr.setAddArr(velocities, i, vertices, i, vertices_cache, i, -1f);
            ArrayOpr.mulScale(velocities, i, sceneData.damping/sceneData.dt);
        }
    }

    public void initConstraints(){
        for (BaseConstraint constraint : constraints) {
            constraint.init();
        }
    }

    public void preUpdateProject(){
        for (BaseConstraint constraint : constraints) {
            constraint.preUpdateProject();
        }
    }

    public void project(int level){
        for(int i=0; i<constraints.size(); i++){
            if(constraintLevels.get(i) == level){
                constraints.get(i).project();
            }
        }
    }

    public void project(){
        for (BaseConstraint constraint : constraints) {
            constraint.project();
        }
    }

    public void collisionY(double groundY){
        for(int i=0; i<n_verts; i++){
            if(vertices[i*2+1] < groundY){
                vertices[i*2+1] = groundY + (double)Math.random() * 1e-4f;
            }
        }
    }
}
