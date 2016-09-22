package us.ihmc.robotEnvironmentAwareness.geometry;

import javax.vecmath.Point2d;

/**
 * A pocket is a concave region.
 * It has different properties:
 *  - It has a unique bridge
 *  - It has a deepest vertex (straight-line criterion)
 *  - It has a maximum depth. 
 * @author Sylvain
 *
 */
public class ConcaveHullPocket
{
   private int startBridgeIndex;
   private int endBridgeIndex;

   private Point2d startBridgeVertex;
   private Point2d endBridgeVertex;

   private int deepestVertexIndex;
   private Point2d deepestVertex;

   private double maxDepth;

   public ConcaveHullPocket()
   {
      clear();
   }

   public void clear()
   {
      startBridgeIndex = -1;
      endBridgeIndex = -1;

      startBridgeVertex = null;
      endBridgeVertex = null;

      clearDepthParameters();
   }

   public void clearDepthParameters()
   {
      deepestVertexIndex = -1;
      deepestVertex = null;

      maxDepth = Double.NEGATIVE_INFINITY;
   }

   public void setBridgeIndices(int startBridgeIndex, int endBridgeIndex)
   {
      this.startBridgeIndex = startBridgeIndex;
      this.endBridgeIndex = endBridgeIndex;
   }

   public void setBridgeVertices(Point2d startBridgeVertex, Point2d endBridgeVertex)
   {
      this.startBridgeVertex = startBridgeVertex;
      this.endBridgeVertex = endBridgeVertex;
   }

   public void setDeepestVertexIndex(int deepestVertexIndex)
   {
      this.deepestVertexIndex = deepestVertexIndex;
   }

   public void setDeepestVertex(Point2d deepestVertex)
   {
      this.deepestVertex = deepestVertex;
   }

   public void setMaxDepth(double maxDepth)
   {
      this.maxDepth = maxDepth;
   }

   public int getStartBridgeIndex()
   {
      return startBridgeIndex;
   }

   public int getEndBridgeIndex()
   {
      return endBridgeIndex;
   }

   public Point2d getStartBridgeVertex()
   {
      return startBridgeVertex;
   }

   public Point2d getEndBridgeVertex()
   {
      return endBridgeVertex;
   }

   public int getDeepestVertexIndex()
   {
      return deepestVertexIndex;
   }

   public Point2d getDeepestVertex()
   {
      return deepestVertex;
   }

   public double getMaxDepth()
   {
      return maxDepth;
   }
}
