package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

public class LidarFrameViewer
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine lidarPose = new Affine();

   private final AtomicReference<Affine> lastAffine = new AtomicReference<>();
   private final AnimationTimer lidarUpdater;

   private final Group root = new Group();

   public LidarFrameViewer()
   {
      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(lidarPose);
      root.getChildren().add(lidarCoordinateSystem);

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

   public PacketConsumer<LidarPosePacket> createLidarPosePacketConsumer()
   {
      return this::handlePacket;
   }

   private void handlePacket(LidarPosePacket packet)
   {
      if (packet == null)
         return;
      Quat4d orientation = packet.getOrientation();
      Point3d position = packet.getPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
   }

   public Node getRoot()
   {
      return root;
   }
}
