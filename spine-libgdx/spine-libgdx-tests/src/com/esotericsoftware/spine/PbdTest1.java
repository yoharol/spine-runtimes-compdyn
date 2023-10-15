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

public class PbdTest1 extends ApplicationAdapter{

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

    @Override
    public void create (){
        camera = new OrthographicCamera();
        batch = new TwoColorPolygonBatch();
        renderer = new SkeletonRenderer();
        renderer.setPremultipliedAlpha(true);
        debugRenderer = new SkeletonRendererDebug();
        debugRenderer.setBoundingBoxes(true);
        debugRenderer.setRegionAttachments(true);

        atlas = new TextureAtlas(Gdx.files.internal("fish/fish.atlas"));
        SkeletonJson json = new SkeletonJson(atlas);
        json.setScale(0.15f);
        SkeletonData skeletonData = json.readSkeletonData(Gdx.files.internal("fish/fish.json"));
        skeleton = new Skeleton(skeletonData);
        skeleton.setPosition(250, 250);

        AnimationStateData stateData = new AnimationStateData(skeletonData); // Defines mixing (crossfading) between animations.
        stateData.setMix("swing", "swing", 0f);
        state = new AnimationState(stateData);
        state.setAnimation(0, "swing", true);
        state.setTimeScale(1.0f); // Slow all animations down to 50% speed.

        font = new BitmapFont(Gdx.files.internal("fonts/Amble-Regular-26.fnt"), Gdx.files.internal("fonts/Amble-Regular-26.png"), false);
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
        lbsData = new LbsData(meshData, bones, vertices, boneObjs, n_verts);

        // set up pbd framework
        deformMesh = new DeformMesh(meshData);
        sceneData = new PhysicsSceneData();
        sceneData.setGravity(0, -100f);
        pbdFramework = new PbdFramework(sceneData, deformMesh);
        pbdFramework.addConstraint(new DeformConstraint(deformMesh, sceneData, 1e-8f, 1e-8f));
        pbdFramework.initConstraints();

        // lbsData = new LbsData(bones, boneObjs);
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
    public void render() {
        PhysicsUpdate();

        state.update(Gdx.graphics.getDeltaTime()); // Update the animation time.

        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        state.apply(skeleton); // Poses skeleton using current animations. This sets the bones' local SRT.
        skeleton.updateWorldTransform(); // Uses the bones' local SRT to compute their world SRT.

        // Configure the camera, SpriteBatch, and SkeletonRendererDebug.
        camera.update();
        batch.getProjectionMatrix().set(camera.combined);
        debugRenderer.getShapeRenderer().setProjectionMatrix(camera.combined);

        batch.begin();
        renderer.draw(batch, skeleton); // Draw the skeleton images.
        batch.end();

        debugRenderer.draw(skeleton); // Draw debug lines.

        // show the world rotation of the bone

        spriteBatch.begin();
        // debug text
        font.setColor(Color.WHITE);
        font.getData().setScale(1.0f);
        // font.draw(spriteBatch, "1 2 3", 50, 50);
        spriteBatch.end();

        shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        short[] indices = meshData.getIndices();
        double[] vertices = meshData.getVertices();
        for (int i=0; i<indices.length / 3; i++){
            short i1 = indices[i*3];
            short i2 = indices[i*3+1];
            short i3 = indices[i*3+2];
            float x1 = (float)vertices[i1*2];
            float y1 = (float)vertices[i1*2+1];
            float x2 = (float)vertices[i2*2];
            float y2 = (float)vertices[i2*2+1];
            float x3 = (float)vertices[i3*2];
            float y3 = (float)vertices[i3*2+1];
            shapeRenderer.triangle(x1, y1, x2, y2, x3, y3);
        }
        shapeRenderer.end();

    }

    public void resize (int width, int height) {
        camera.setToOrtho(false); // Update camera with new size.
    }

    public void dispose () {
        atlas.dispose();
        font.dispose();
        batch.dispose();
    }

    public static void main (String[] args) throws Exception {
        new Lwjgl3Application(new PbdTest1());
    }



}
