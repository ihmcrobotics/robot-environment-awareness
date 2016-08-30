package us.ihmc.robotEnvironmentAwareness.ui;

import java.util.concurrent.ConcurrentLinkedDeque;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.transform.Affine;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.robotEnvironmentAwareness.simulation.LidarPosePacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class LidarFrameViewer extends Group
{
   private final JavaFXCoordinateSystem lidarCoordinateSystem;
   private final Affine lidarPose = new Affine();

   private final ConcurrentLinkedDeque<Affine> queue = new ConcurrentLinkedDeque<>();

   public LidarFrameViewer()
   {
      lidarCoordinateSystem = new JavaFXCoordinateSystem(0.1);
      lidarCoordinateSystem.getTransforms().add(lidarPose);
      getChildren().add(lidarCoordinateSystem);

      new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (!queue.isEmpty())
            {
               Affine newAffine = queue.pollLast();
               
               if (newAffine != null)
                  lidarPose.setToTransform(newAffine);
            }
         }
      }.start();
   }

   public PacketConsumer<LidarPosePacket> createLidarPosePacketConsumer()
   {
      PacketConsumer<LidarPosePacket> packetConsumer = new PacketConsumer<LidarPosePacket>()
      {
         private final RigidBodyTransform localTransform = new RigidBodyTransform();

         @Override
         public void receivedPacket(LidarPosePacket packet)
         {
            if (packet == null || queue.size() > 5)
               return;

            Quat4d orientation = packet.getOrientation();
            Point3d position = packet.getPosition();

            localTransform.setRotation(orientation);
            localTransform.setTranslation(position.getX(), position.getY(), position.getZ());
            queue.addLast(JavaFXTools.convertRigidBodyTransformToAffine(localTransform));
         }
      };
      return packetConsumer;
   }
}
