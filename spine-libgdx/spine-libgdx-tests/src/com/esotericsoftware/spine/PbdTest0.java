package com.esotericsoftware.spine;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.esotericsoftware.spine.pbd.*;

public class PbdTest0  extends ApplicationAdapter {

    ShapeRenderer shapeRenderer;
    OrthographicCamera camera;

    MeshData meshData;
    DeformMesh deformMesh;
    PhysicsSceneData sceneData;
    PbdFramework pbdFramework;

    @Override
    public void create () {
        sceneData = new PhysicsSceneData();
        sceneData.setGravity(0, -100f);
        // sceneData.setDamping(0.99f);

        shapeRenderer = new ShapeRenderer();
        camera = new OrthographicCamera();
        camera.setToOrtho(false, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
        double[] vertices = {200, 200, 200, 350, 350, 200, 350, 350};
        short[] indices = {0, 1, 2, 1, 2, 3};

        meshData = new MeshData(vertices, indices);
        deformMesh = new DeformMesh(meshData);
        pbdFramework = new PbdFramework(sceneData, deformMesh);
        pbdFramework.addConstraint(new DeformConstraint(deformMesh, sceneData, 1e-8f, 1e-8f));
        pbdFramework.initConstraints();
    }

    void PhysicsUpdate(){
        for(int i=0; i< sceneData.iterations; i++) {
            pbdFramework.makePrediction();
            pbdFramework.preUpdateProject();
            for(int k=0; k< sceneData.solver_steps; k++){
                pbdFramework.project();
            }
            pbdFramework.collisionY(0);
            pbdFramework.updateVelocity();
        }
    }

    @Override
    public void render () {
        PhysicsUpdate();
        Gdx.gl.glClear(Gdx.gl.GL_COLOR_BUFFER_BIT);
        camera.update();
        shapeRenderer.setProjectionMatrix(camera.combined);
        shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        float[] vertices = ArrayOpr.convertArray(meshData.getVertices());
        shapeRenderer.triangle(vertices[0], vertices[1], vertices[2], vertices[3], vertices[4], vertices[5]);
        shapeRenderer.triangle(vertices[2], vertices[3], vertices[4], vertices[5], vertices[6], vertices[7]);
        shapeRenderer.end();
    }

    @Override
    public void dispose () {
        shapeRenderer.dispose();
    }

    public static void main (String[] args) throws Exception {
        new Lwjgl3Application(new PbdTest0());
    }
}
