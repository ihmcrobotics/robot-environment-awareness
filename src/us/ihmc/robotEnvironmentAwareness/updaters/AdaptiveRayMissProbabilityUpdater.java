package us.ihmc.robotEnvironmentAwareness.updaters;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree.RayMissProbabilityUpdater;
import us.ihmc.jOctoMap.occupancy.OccupancyParameters;
import us.ihmc.robotics.MathTools;

public class AdaptiveRayMissProbabilityUpdater implements RayMissProbabilityUpdater
{
   private double distanceSquaredToEndThreshold = MathTools.square(0.06);
   private double rayEndMissProbability = 0.47;

   private int normalConsensusThreshold = 10;
   private double dotRayToNormalThreshold = Math.cos(Math.toRadians(150.0));
   private double shallowAngleMissProbability = 0.45;

   @Override
   public double computeRayMissProbability(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, NormalOcTreeNode node, OccupancyParameters parameters)
   {
      Point3d hitLocation = new Point3d();
      node.getHitLocation(hitLocation);

      if (hitLocation.distanceSquared(rayEnd) < distanceSquaredToEndThreshold)
      {
         return rayEndMissProbability;
      }
      else if (node.getNormalConsensusSize() > normalConsensusThreshold && node.isNormalSet())
      {
         Point3d nodeHitLocation = new Point3d();
         Vector3d nodeNormal = new Vector3d();
         node.getHitLocation(nodeHitLocation);
         node.getNormal(nodeNormal);

         if (Math.abs(rayDirection.dot(nodeNormal)) < dotRayToNormalThreshold)
            return shallowAngleMissProbability;
         else
            return parameters.getMissProbability();
      }
      else
      {
         return parameters.getMissProbability();
      }
   }
}
