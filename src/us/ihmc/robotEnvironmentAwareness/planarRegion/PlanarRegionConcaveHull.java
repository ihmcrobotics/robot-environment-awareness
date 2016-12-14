package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;

public class PlanarRegionConcaveHull
{
   private final OcTreeNodePlanarRegion ocTreeNodePlanarRegion;
   private final ConcaveHullCollection concaveHullCollection;

   public PlanarRegionConcaveHull(OcTreeNodePlanarRegion planarRegion, ConcaveHullCollection concaveHullCollection)
   {
      this.ocTreeNodePlanarRegion = planarRegion;
      this.concaveHullCollection = concaveHullCollection;
   }

   public int getRegionId()
   {
      return ocTreeNodePlanarRegion.getId();
   }

   public OcTreeNodePlanarRegion getOcTreeNodePlanarRegion()
   {
      return ocTreeNodePlanarRegion;
   }

   public ConcaveHullCollection getConcaveHullCollection()
   {
      return concaveHullCollection;
   }
}
