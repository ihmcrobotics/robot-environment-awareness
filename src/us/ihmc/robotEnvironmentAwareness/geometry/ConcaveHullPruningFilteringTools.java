package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.*;
import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.increment;
import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.removeAllExclusive;
import static us.ihmc.robotics.geometry.GeometryTools.computeTriangleArea;
import static us.ihmc.robotics.geometry.GeometryTools.isPointOnLeftSideOfLine;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.GeometryTools;

/**
 * This class gathers different filters on a concave hull.
 * The filters aim to reduce the hull complexity by removing less significant vertices.
 * The filters gathered here only tend to reduce the area of the concave hull.
 * 
 * @author Sylvain
 *
 */
public class ConcaveHullPruningFilteringTools
{
   /**
    * Filter out vertices that create "peaks" or barely stick out the line described by the previous and next vertices.
    * Peaks are identified by a threshold on the angle between two consecutive edges.
    * Only convex peaks or shallow angles are removed, meaning this filter only reduces the area of the concave hull.
    * @param shallowAngleThreshold should be a small positive angle in radians. 0 will not remove any vertex.
    * @param peakAngleThreshold should be close to {@link Math#PI}.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter. 
    * @return the number of vertices removed.
    */
   public static int filterOutPeaksAndShallowAngles(double shallowAngleThreshold, double peakAngleThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      shallowAngleThreshold = - Math.abs(shallowAngleThreshold);
      peakAngleThreshold = - Math.abs(peakAngleThreshold);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         if (!ConcaveHullTools.isConvexAtVertex(currentIndex, concaveHullVerticesToFilter))
         {
            currentIndex++;
            continue;
         }

         double angle = ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(currentIndex, concaveHullVerticesToFilter);
         boolean isAngleWithinThresholds = angle < peakAngleThreshold || angle > shallowAngleThreshold;

         if (isAngleWithinThresholds && !isVertexPreventingKink(currentIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(currentIndex);
            numberOfVerticesRemoved++;
         }
         else
         {
            currentIndex++;
         }
      }
      return numberOfVerticesRemoved;
   }

   public static int filterOutShallowVertices(double percentageThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2d a = concaveHullVerticesToFilter.get(0);
      Point2d b = concaveHullVerticesToFilter.get(1);
      Point2d c = concaveHullVerticesToFilter.get(2);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // If convex at b, then b should be on the outside of ac => on the left of the vector ac.
         boolean isConvex = isPointOnLeftSideOfLine(b, a, c);

         int nextIndex = increment(currentIndex, concaveHullVerticesToFilter);
         if (isConvex && a.distance(c) / (a.distance(b) + b.distance(c)) > percentageThreshold && !isVertexPreventingKink(nextIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(nextIndex);
            b = c;
            numberOfVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get(increment(nextIndex, concaveHullVerticesToFilter));
      }

      return numberOfVerticesRemoved;
   }

   public static int filterOutGroupsOfShallowVertices(double percentageThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      Point2d firstVertex;
      Point2d intermediateVertex;
      Point2d lastVertex;

      double perimeter = 0.0;
      double cuttingDistance = 0.0;
      double ratio = 0.0;

      for (int startCutIndex = 0; startCutIndex < concaveHullVerticesToFilter.size();)
      {
         firstVertex = concaveHullVerticesToFilter.get(startCutIndex);
         int intermediateIndex = increment(startCutIndex, concaveHullVerticesToFilter);
         intermediateVertex = concaveHullVerticesToFilter.get(intermediateIndex);
         int lastVertexIndex = increment(intermediateIndex, concaveHullVerticesToFilter);
         lastVertex = concaveHullVerticesToFilter.get(lastVertexIndex);

         perimeter = firstVertex.distance(intermediateVertex);
         perimeter += intermediateVertex.distance(lastVertex);
         cuttingDistance = firstVertex.distance(lastVertex);

         ratio = cuttingDistance / perimeter;

         // Cannot cut, skipping to the next vertex
         if (ratio < percentageThreshold)
         {
            startCutIndex++;
            continue;
         }

         // Can cut at least one vertex, going further to see if more can be removed
         int endCutIndex = lastVertexIndex;

         while (ratio > percentageThreshold)
         {
            intermediateVertex = lastVertex;
            endCutIndex = lastVertexIndex;
            lastVertexIndex = increment(lastVertexIndex, concaveHullVerticesToFilter);
            lastVertex = concaveHullVerticesToFilter.get(lastVertexIndex);

            perimeter += intermediateVertex.distance(lastVertex);
            cuttingDistance = firstVertex.distance(lastVertex);

            ratio = cuttingDistance / perimeter;
         }

         int firstRemovableVertexIndex = increment(startCutIndex, concaveHullVerticesToFilter);

         for (int checkIndex = firstRemovableVertexIndex; checkIndex != endCutIndex;)
         {
            Point2d vertexToCheck = concaveHullVerticesToFilter.get(checkIndex);

            if (!GeometryTools.isPointOnLeftSideOfLine(vertexToCheck, firstVertex, lastVertex))
            {
               // Reducing the cutting line
               endCutIndex = checkIndex;
               if (endCutIndex == startCutIndex)
               { // Nothing to cut anymore
                  break;
               }
               // Starting over the check
               checkIndex = startCutIndex;
            }
            else
            {
               checkIndex = increment(checkIndex, concaveHullVerticesToFilter);
            }
         }

         // Nothing to cut, skipping.
         if (endCutIndex == startCutIndex)
         {
            startCutIndex++;
            continue;
         }

         // Removing the vertices.
         for (int index = firstRemovableVertexIndex; index != endCutIndex;)
         {
            index = increment(index, concaveHullVerticesToFilter);
            concaveHullVerticesToFilter.remove(firstRemovableVertexIndex);
            numberOfVerticesRemoved++;
         }
      }

      return numberOfVerticesRemoved;
   }

   /**
    * By looking at each triplet of successive vertices, a triangle is formed.
    * The area of the triangle is computed and if below the given threshold, the middle vertex is removed.
    * @param areaThreshold a vertex forming a triangle with an area smaller than that parameter will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter. 
    * @return the number of vertices removed.
    */
   public static int filterOutSmallTriangles(double areaThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2d a = concaveHullVerticesToFilter.get(0);
      Point2d b = concaveHullVerticesToFilter.get(1);
      Point2d c = concaveHullVerticesToFilter.get(2);

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // If convex at b, then b should be on the outside of ac => on the left of the vector ac.
         boolean isConvex = isPointOnLeftSideOfLine(b, a, c);
         int nextIndex = increment(currentIndex, concaveHullVerticesToFilter);
         if (isConvex && computeTriangleArea(a, b, c) < areaThreshold && !isVertexPreventingKink(nextIndex, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove(nextIndex);
            b = c;
            numberOfVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get(increment(nextIndex, concaveHullVerticesToFilter));
      }

      return numberOfVerticesRemoved;
   }

   public static int flattenShallowPockets(double depthThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      ConcaveHullPocket pocket = new ConcaveHullPocket();

      Vector2d shift = new Vector2d();

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size(); currentIndex++)
      {
         if (isConvexAtVertex(currentIndex, concaveHullVerticesToFilter))
            continue;

         boolean success = computeConcaveHullPocket(currentIndex, pocket, concaveHullVerticesToFilter);
         if (!success)
            continue;

         double maxDepth = pocket.getMaxDepth();
         if (maxDepth > depthThreshold)
            continue;


         Point2d startBridgeVertex = pocket.getStartBridgeVertex();
         Point2d endBridgeVertex = pocket.getEndBridgeVertex();
         shift.sub(endBridgeVertex, startBridgeVertex);
         shift.normalize();
         // Rotate to the right
         shift.set(shift.getY(), -shift.getX());
         shift.scale(maxDepth);

         System.out.println("start bridge: " + pocket.getStartBridgeIndex() + " " + pocket.getStartBridgeVertex() + ", end bridge: " + pocket.getEndBridgeIndex() + " " + pocket.getEndBridgeVertex() + ", shift: " + shift + ", size: " + concaveHullVerticesToFilter.size());
         startBridgeVertex.add(shift);
         endBridgeVertex.add(shift);

         int startBridgeVertexIndex = pocket.getStartBridgeIndex();
         int endBridgeVertexIndex = pocket.getEndBridgeIndex();
         numberOfVerticesRemoved += removeAllExclusive(startBridgeVertexIndex, endBridgeVertexIndex, concaveHullVerticesToFilter);
      }

      return numberOfVerticesRemoved;
   }

   /**
    * Removes vertices to filter short edges. Only convex vertices are removed, meaning the polygon area can only decrease when calling this method.
    * @param concaveAngleLimit threshold to define a concavity. 0 rad being flat, negative convex, positive concave.
    * @param lengthThreshold any edge shorter than that will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutShortEdges(double lengthThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int numberOfVerticesRemoved = 0;
      double lengthThresholdSquared = lengthThreshold * lengthThreshold;

      Vector2d edgeVector = new Vector2d();
      Vector2d previousEdgeVector = new Vector2d();
      Vector2d nextEdgeVector = new Vector2d();

      for (int beforeEdgeVertexIndex = 0; beforeEdgeVertexIndex < concaveHullVerticesToFilter.size();)
      {
         int firstEdgeVertexIndex = increment(beforeEdgeVertexIndex, concaveHullVerticesToFilter);
         int secondEdgeVertexIndex = increment(firstEdgeVertexIndex, concaveHullVerticesToFilter);
         int afterEdgeVertexIndex = increment(secondEdgeVertexIndex, concaveHullVerticesToFilter);

         Point2d firstEdgeVertex = concaveHullVerticesToFilter.get(firstEdgeVertexIndex);
         Point2d secondEdgeVertex = concaveHullVerticesToFilter.get(secondEdgeVertexIndex);
         double edgeLengthSquared = firstEdgeVertex.distanceSquared(secondEdgeVertex);

         // This is a long edge, skip.
         if (edgeLengthSquared > lengthThresholdSquared)
         {
            beforeEdgeVertexIndex++;
            continue;
         }

         Point2d beforeEdgeVertex = concaveHullVerticesToFilter.get(beforeEdgeVertexIndex);
         Point2d afterEdgeVertex = concaveHullVerticesToFilter.get(afterEdgeVertexIndex);

         boolean isFirstEdgeVertexConvex = isPointOnLeftSideOfLine(firstEdgeVertex, beforeEdgeVertex, secondEdgeVertex);
         boolean isSecondEdgeVertexConvex = isPointOnLeftSideOfLine(secondEdgeVertex, firstEdgeVertex, afterEdgeVertex);

         // Both vertices are in a concavity, cannot remove any of them without expanding the concave hull.
         if (!isFirstEdgeVertexConvex && !isSecondEdgeVertexConvex)
         {
            beforeEdgeVertexIndex++;
         }
         // Only one of the two vertices can be removed 
         else if (isFirstEdgeVertexConvex != isSecondEdgeVertexConvex)
         {
            // Is it the first one?
            if (isFirstEdgeVertexConvex)
            {
               // Check if removing the vertex would create a kink in the polygon
               if (isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Skip vertex
                  beforeEdgeVertexIndex++;
               }
               else
               { // It is safe to remove the vertex
                  concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
            }
            // Reaching here means only the second vertex could be removed. Check if removing it would create a kink.
            else
            {
               // Check if removing the vertex would create a kink in the polygon
               if (isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Skip vertex
                  beforeEdgeVertexIndex++;
               }
               else
               { // Safe to remove the second vertex 
                  concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
            }
         }
         else // Here, any of the two vertices can be removed.
         {
            edgeVector.sub(secondEdgeVertex, firstEdgeVertex);
            edgeVector.normalize();

            previousEdgeVector.sub(firstEdgeVertex, beforeEdgeVertex);
            previousEdgeVector.normalize();

            nextEdgeVector.sub(afterEdgeVertex, secondEdgeVertex);
            nextEdgeVector.normalize();

            double edgeDotNextEdge = edgeVector.dot(nextEdgeVector);
            double previousEdgeDotEdge = previousEdgeVector.dot(edgeVector);

            // Pick the vertex that affects the least the polygon when removing it.
            // Basically looking at which vertex is the "flatter"
            if (previousEdgeDotEdge > edgeDotNextEdge)
            { // The first vertex is the best option. Still need to check if removing it would create a kink.
               if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Can remove the first vertex
                  concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
               else
               { // Cannot remove the first vertex. Check if the second vertex can be removed.
                  if (!isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the second vertex.
                     concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                     numberOfVerticesRemoved++;
                  }
                  else
                  { // Cannot removed the second vertex either, skip.
                     beforeEdgeVertexIndex++;
                  }
               }
            }
            else
            { // The second vertex is the best option, checking if removing it would create a kink.
               if (!isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Can remove the second vertex
                  concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                  numberOfVerticesRemoved++;
               }
               else
               { // Cannot remove the second vertex. Check if the first vertex can be removed.
                  if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the first vertex.
                     concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                     numberOfVerticesRemoved++;
                  }
                  else
                  { // Cannot removed the first vertex either, skip.
                     beforeEdgeVertexIndex++;
                  }
               }
            }
         }
      }
      return numberOfVerticesRemoved;
   }
}
