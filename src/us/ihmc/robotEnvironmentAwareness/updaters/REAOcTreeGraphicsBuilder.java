package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

public class REAOcTreeGraphicsBuilder
{
   private final NormalOcTree octree;
   private final RegionFeaturesProvider regionFeaturesProvider;

   private final AtomicReference<Boolean> isOcTreeRequested;

   private final AtomicReference<Boolean> isBoundingBoxRequested;

   private final REAMessager reaMessager;
   private final Vector3d size = new Vector3d();
   private final Point3d center = new Point3d();

   public REAOcTreeGraphicsBuilder(NormalOcTree octree, RegionFeaturesProvider regionFeaturesProvider, REAMessager reaMessager)
   {
      this.octree = octree;
      this.regionFeaturesProvider = regionFeaturesProvider;
      this.reaMessager = reaMessager;
      isOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestOctree, false);
      isBoundingBoxRequested = reaMessager.createInput(REAModuleAPI.RequestBoundingBox, false);
   }

   public void update()
   {
      if (isOcTreeRequested.getAndSet(false))
      {
         NormalOcTreeMessage normalOcTreeMessage = OcTreeMessageConverter.convertToMessage(octree, regionFeaturesProvider.getOcTreePlanarRegions());
         reaMessager.submitMessage(REAModuleAPI.OcTreeState, normalOcTreeMessage);
      }

      if (isBoundingBoxRequested.getAndSet(false))
      {
         BoxMessage boxMessage = new BoxMessage();
         OcTreeBoundingBoxWithCenterAndYaw boundingBox = (OcTreeBoundingBoxWithCenterAndYaw) octree.getBoundingBox();

         if (boundingBox != null)
         {
            boundingBox.getLocalSize(size);
            boundingBox.getCenterCoordinate(center);
            double yaw = boundingBox.getYaw();

            boxMessage.setSize(size);
            boxMessage.setCenter(center);
            boxMessage.setOrientation(new Quat4d(0.0, 0.0, Math.sin(0.5 * yaw), Math.cos(0.5 * yaw)));
         }
         reaMessager.submitMessage(REAModuleAPI.BoundingBoxState, boxMessage);
      }
   }
}
