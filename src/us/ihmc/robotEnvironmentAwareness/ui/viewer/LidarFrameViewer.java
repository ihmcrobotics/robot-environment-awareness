package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class LidarFrameViewer
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine lidarPose = new Affine();

   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();
   private final AnimationTimer lidarUpdater;

   private final Group root = new Group();

   public LidarFrameViewer(REAMessager reaMessager)
   {
      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(lidarPose);
      root.getChildren().add(lidarCoordinateSystem);

      reaMessager.registerListener(REAModuleAPI.LidarScanState, this::handleMessage);

      lidarUpdater = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            updateLidarPose();
         }
      };
   }

   public void start()
   {
      lidarUpdater.start();
   }

   public void stop()
   {
      lidarUpdater.stop();
   }

   private void updateLidarPose()
   {
      Affine affine = lastAffine.getAndSet(null);
      if (affine != null)
         lidarPose.setToTransform(affine);
   }

   private void handleMessage(LidarScanMessage lidarScanMessage)
   {
      if (lidarScanMessage == null)
         return;
      Quat4f orientation = lidarScanMessage.getLidarOrientation();
      Point3f position = lidarScanMessage.getLidarPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
   }

   public Node getRoot()
   {
      return root;
   }
}
