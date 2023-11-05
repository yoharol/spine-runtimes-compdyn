
package com.esotericsoftware.spine;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.utils.Array;
import com.esotericsoftware.spine.attachments.MeshAttachment;
import com.esotericsoftware.spine.pbd.*;
import com.esotericsoftware.spine.utils.TwoColorPolygonBatch;

public class PbdTest2 extends ApplicationAdapter{
    // gdx and spine stuff
    OrthographicCamera camera;
    TwoColorPolygonBatch batch;
    SkeletonRenderer renderer;
    SkeletonRendererDebug debugRenderer;
    BitmapFont font;
    SpriteBatch spriteBatch;
    ShapeRenderer shapeRenderer;
    TextureAtlas atlas;
    Skeleton skeleton;
    AnimationState state;
    Slot slot;
    MeshAttachment meshAttachment;

    // data
    MeshData meshData;
    LbsData lbsData;

    // simulation stuff
    DeformMesh deformMesh;
    PhysicsSceneData sceneData;
    PbdFramework pbdFramework;
    DeformConstraint deformConstraint;
    ShapeConstraint shapeConstraint;

    @Override
    public void create (){
        camera = new OrthographicCamera();
        batch = new TwoColorPolygonBatch();
        renderer = new SkeletonRenderer();
        renderer.setPremultipliedAlpha(true);
        debugRenderer = new SkeletonRendererDebug();
        debugRenderer.setBoundingBoxes(true);
        debugRenderer.setRegionAttachments(true);

        atlas = new TextureAtlas(Gdx.files.internal("fishNbones/fish.atlas"));
        SkeletonJson json = new SkeletonJson(atlas);
        json.setScale(0.15f);
        SkeletonData skeletonData = json.readSkeletonData(Gdx.files.internal("fishNbones/fish.json"));
        skeleton = new Skeleton(skeletonData);
        skeleton.setPosition(250, 250);

        AnimationStateData stateData = new AnimationStateData(skeletonData); // Defines mixing (crossfading) between animations.
        stateData.setMix("swing", "swing", 0f);
        state = new AnimationState(stateData);
        state.setAnimation(0, "swing", true);
        state.setTimeScale(1.5f); // Slow all animations down to 50% speed.

        spriteBatch = new SpriteBatch();
        shapeRenderer = new ShapeRenderer();

        slot = skeleton.findSlot("fish");
        meshAttachment = (MeshAttachment) skeleton.getAttachment("fish", "fish");
        skeleton.setBonesToSetupPose();
        skeleton.updateWorldTransform();

        // set up mesh and lbs data
        double[] vertices = ArrayOpr.convertArray(meshAttachment.getVertices());
        short[] indices = meshAttachment.getTriangles();
        int[] bones = meshAttachment.getBones();
        Array<Bone> boneObjs = slot.getSkeleton().getBones();
        int n_verts = meshAttachment.getWorldVerticesLength()>>1;
        int n_faces = indices.length/3;
        float[] fWorldVertices = new float[n_verts*2];
        meshAttachment.computeWorldVertices(slot, 0, n_verts*2, fWorldVertices, 0, 2);
        double[] worldVertices = ArrayOpr.convertArray(fWorldVertices);
        meshData = new MeshData(worldVertices, indices);

        // set up pbd framework
        double damping = 0.987;
        int solver_iterations = 6;

        deformMesh = new DeformMesh(meshData);
        lbsData = new LbsData(deformMesh, bones, vertices, boneObjs, n_verts);
        sceneData = new PhysicsSceneData();
        // sceneData.setGravity(0, 0f);
        sceneData.setGravity(0, -1f);
        sceneData.setDamping(damping);
        sceneData.setFps(60, solver_iterations, 1);
        pbdFramework = new PbdFramework(sceneData, deformMesh);

        // set up constraints
        deformConstraint = new DeformConstraint(deformMesh, sceneData, 1e-4, 1e-4);
        shapeConstraint = new ShapeConstraint(deformMesh, lbsData);
    }

    void PhysicsUpdate(){
        lbsData.updateLbsVerts(meshAttachment, slot, meshData.getScale());
        for(int i=0; i< sceneData.iterations; i++) {
            pbdFramework.makePrediction();
            deformConstraint.preUpdateProject();
            deformConstraint.project();
            shapeConstraint.project_single_bone(1);

            int max_bones = lbsData.getN_bones();
            for(int j=2; j<max_bones; j++){
                Bone b = lbsData.getBone(j);
                b.updateWorldTransform();
                Mat2x2 A = lbsData.inverseMixed(j, 1.0);
                b.setA((float)A.a());
                b.setB((float)A.b());
                b.setC((float)A.c());
                b.setD((float)A.d());
                b.updateAppliedTransform();
            }

            pbdFramework.updateVelocity();
        }
    }

    @Override
    public void render() {

        state.update(Gdx.graphics.getDeltaTime()); // Update the animation time.

        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        state.apply(skeleton); // Poses skeleton using current animations. This sets the bones' local SRT.
        skeleton.updateWorldTransform(); // Uses the bones' local SRT to compute their world SRT.

        PhysicsUpdate();

        // Configure the camera, SpriteBatch, and SkeletonRendererDebug.
        camera.update();
        batch.getProjectionMatrix().set(camera.combined);
        debugRenderer.getShapeRenderer().setProjectionMatrix(camera.combined);

        batch.begin();
        renderer.draw(batch, skeleton); // Draw the skeleton images.
        batch.end();

        debugRenderer.draw(skeleton); // Draw debug lines.

        // show the world rotation of the bone

        // render the background physics mesh
        /*shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        short[] indices = meshData.getIndices();
        double[] vertices = deformMesh.getVertices();
        // double[] vertices = deformMesh.getRefVertices();
        // double[] vertices = lbsData.getRigVerts();
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
        shapeRenderer.end();*/

    }

    public void resize (int width, int height) {
        camera.setToOrtho(false); // Update camera with new size.
    }

    public void dispose () {
        atlas.dispose();
        batch.dispose();
    }

    public static void main (String[] args) throws Exception {
        new Lwjgl3Application(new PbdTest2());
    }

}
