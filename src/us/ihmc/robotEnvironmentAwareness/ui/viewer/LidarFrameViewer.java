package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class LidarFrameViewer extends AnimationTimer
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine lidarPose = new Affine();

   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();

   private final Group root = new Group();
   private final REAUIMessager uiMessager;

   public LidarFrameViewer(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(lidarPose);
      root.getChildren().add(lidarCoordinateSystem);

      uiMessager.registerTopicListener(REAModuleAPI.LidarScanState, this::handleMessage);
      uiMessager.registerModuleConnectionStateListener(new NetStateListener()
      {
         @Override
         public void disconnected()
         {
            stop();
         }
         
         @Override
         public void connected()
         {
            start();
         }
      });
   }

   @Override
   public void handle(long now)
   {
      Affine affine = lastAffine.getAndSet(null);
      if (affine != null)
         lidarPose.setToTransform(affine);
      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestLidarScan);
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
