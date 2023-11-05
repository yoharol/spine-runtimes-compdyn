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
        sceneData.setGravity(0, -10f);
        sceneData.setDamping(0.995f);

        shapeRenderer = new ShapeRenderer();
        camera = new OrthographicCamera();
        camera.setToOrtho(false, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
        double[] vertices = {200, 200, 200, 350, 350, 200, 350, 350};
        short[] indices = {0, 1, 2, 1, 2, 3};

        meshData = new MeshData(vertices, indices);
        deformMesh = new DeformMesh(meshData);
        pbdFramework = new PbdFramework(sceneData, deformMesh);
        pbdFramework.addConstraint(new DeformConstraint(deformMesh, sceneData, 1e-3f, 1e-3f));
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
        short[] indices = meshData.getIndices();
        double[] vertices = deformMesh.getVertices();
        double scale = meshData.getScale();
        for (int i=0; i<indices.length / 3; i++){
            short i1 = indices[i*3];
            short i2 = indices[i*3+1];
            short i3 = indices[i*3+2];
            float x1 = (float)(vertices[i1*2]*scale);
            float y1 = (float)(vertices[i1*2+1]*scale);
            float x2 = (float)(vertices[i2*2]*scale);
            float y2 = (float)(vertices[i2*2+1]*scale);
            float x3 = (float)(vertices[i3*2]*scale);
            float y3 = (float)(vertices[i3*2+1]*scale);
            shapeRenderer.triangle(x1, y1, x2, y2, x3, y3);
        }
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
