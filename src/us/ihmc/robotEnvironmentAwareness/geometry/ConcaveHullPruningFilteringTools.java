package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.isVertexPreventingKink;
import static us.ihmc.robotics.geometry.GeometryTools.computeTriangleArea;
import static us.ihmc.robotics.geometry.GeometryTools.cross;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

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
    * @param concaveAngleLimit threshold to define a concavity. 0 rad being flat, negative convex, positive concave.
    * @param shallowAngleThreshold should be a small positive angle in radians. 0 will not remove any vertex.
    * @param peakAngleThreshold should be close to {@link Math#PI}.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter. 
    * @return the number of vertices removed.
    */
   public static int filterOutPeaksAndShallowAngles(double concaveAngleLimit, double shallowAngleThreshold, double peakAngleThreshold,
         List<Point2d> concaveHullVerticesToFilter)
   {
      int nVerticesRemoved = 0;
      double concaveLimit = Math.sin(concaveAngleLimit);
      double cosPeakAngle = Math.cos(peakAngleThreshold);
      double cosShallowAngle = Math.cos(shallowAngleThreshold);

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2d a = concaveHullVerticesToFilter.get(0);
      Point2d b = concaveHullVerticesToFilter.get(1);
      Point2d c = concaveHullVerticesToFilter.get(2);

      Vector2d ab = new Vector2d();
      Vector2d ac = new Vector2d();
      Vector2d bc = new Vector2d();
      ab.sub(b, a);
      ac.sub(c, a);
      bc.sub(c, b);
      ab.normalize();
      ac.normalize();
      bc.normalize();

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // The cross-product is used to detect if b is on the outside of the line ac, two cases from there:
         // - b is outside, removing it means reducing the concave hull area => good
         // - b is inside, removing it means augmenting the concave hull area => not good here
         double cross = cross(ab, ac);
         // The dot product is used to evaluate the angle at b. If it is too small or too large b may be removed
         double dot = ab.dot(bc);

         if (cross < concaveLimit && (dot < cosPeakAngle || dot > cosShallowAngle) && !isVertexPreventingKink(currentIndex + 1, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove((currentIndex + 1) % concaveHullVerticesToFilter.size());
            b = c;
            ab.set(ac);
            nVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            ab.set(bc);
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get((currentIndex + 2) % concaveHullVerticesToFilter.size());
         ac.sub(c, a);
         ac.normalize();
         bc.sub(c, b);
         bc.normalize();
      }
      return nVerticesRemoved;
   }

   /**
    * By looking at each triplet of successive vertices, a triangle is formed.
    * The area of the triangle is computed and if below the given threshold, the middle vertex is removed.
    * @param concaveAngleLimit threshold to define a concavity. 0 rad being flat, negative convex, positive concave.
    * @param areaThreshold a vertex forming a triangle with an area smaller than that parameter will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter. 
    * @return the number of vertices removed.
    */
   public static int filterOutSmallTriangles(double concaveAngleLimit, double areaThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int nVerticesRemoved = 0;
      double concaveLimit = Math.sin(concaveAngleLimit);      

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2d a = concaveHullVerticesToFilter.get(0);
      Point2d b = concaveHullVerticesToFilter.get(1);
      Point2d c = concaveHullVerticesToFilter.get(2);

      Vector2d ab = new Vector2d();
      Vector2d ac = new Vector2d();
      ab.sub(b, a);
      ac.sub(c, a);
      ab.normalize();
      ac.normalize();

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // The cross-product is used to detect if b is on the outside of the line ac, two cases from there:
         // - b is outside, removing it means reducing the concave hull area => good
         // - b is inside, removing it means augmenting the concave hull area => not good here
         double cross = cross(ab, ac);
         if (cross < concaveLimit && computeTriangleArea(a, b, c) < areaThreshold && !isVertexPreventingKink(currentIndex + 1, concaveHullVerticesToFilter))
         {
            concaveHullVerticesToFilter.remove((currentIndex + 1) % concaveHullVerticesToFilter.size());
            b = c;
            ab.set(ac);
            nVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            ab.sub(b, a);
            ab.normalize();
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get((currentIndex + 2) % concaveHullVerticesToFilter.size());
         ac.sub(c, a);
         ac.normalize();
      }

      return nVerticesRemoved;
   }

   /**
    * Removes vertices to filter short edges. Only convex vertices are removed, meaning the polygon area can only decrease when calling this method.
    * @param concaveAngleLimit threshold to define a concavity. 0 rad being flat, negative convex, positive concave.
    * @param lengthThreshold any edge shorter than that will be removed, if possible.
    * @param concaveHullVerticesToFilter the vertices of the concave hull to filter.
    * @return the number of vertices removed.
    */
   public static int filterOutShortEdges(double concaveAngleLimit, double lengthThreshold, List<Point2d> concaveHullVerticesToFilter)
   {
      int nVerticesRemoved = 0;
      double concaveLimit = Math.sin(concaveAngleLimit);
      double lengthThresholdSquared = lengthThreshold * lengthThreshold;

      Vector2d edgeVector = new Vector2d();
      Vector2d previousEdgeVector = new Vector2d();
      Vector2d nextEdgeVector = new Vector2d();

      for (int beforeEdgeVertexIndex = 0; beforeEdgeVertexIndex < concaveHullVerticesToFilter.size();)
      {
         int firstEdgeVertexIndex = (beforeEdgeVertexIndex + 1) % concaveHullVerticesToFilter.size();
         int secondEdgeVertexIndex = (beforeEdgeVertexIndex + 2) % concaveHullVerticesToFilter.size();
         int afterEdgeVertexIndex = (beforeEdgeVertexIndex + 3) % concaveHullVerticesToFilter.size();

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

         edgeVector.sub(secondEdgeVertex, firstEdgeVertex);
         edgeVector.normalize();
         previousEdgeVector.sub(firstEdgeVertex, beforeEdgeVertex);
         previousEdgeVector.normalize();
         nextEdgeVector.sub(afterEdgeVertex, secondEdgeVertex);
         nextEdgeVector.normalize();

         double edgeCrossNextEdge = cross(edgeVector, nextEdgeVector);
         double previousEdgeCrossEdge = cross(previousEdgeVector, edgeVector);

         boolean isFirstEdgeVertexConvex = previousEdgeCrossEdge < concaveLimit;
         boolean isSecondEdgeVertexConvex = edgeCrossNextEdge < concaveLimit;

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
                  nVerticesRemoved++;
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
                  nVerticesRemoved++;
               }
            }
         }
         else // Here, any of the two vertices can be removed.
         {
            double edgeDotNextEdge = edgeVector.dot(nextEdgeVector);
            double previousEdgeDotEdge = previousEdgeVector.dot(edgeVector);

            // Pick the vertex that affects the least the polygon when removing it.
            // Basically looking at which vertex is the "flatter"
            if (previousEdgeDotEdge > edgeDotNextEdge)
            { // The first vertex is the best option. Still need to check if removing it would create a kink.
               if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
               { // Can remove the first vertex
                  concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                  nVerticesRemoved++;
               }
               else
               { // Cannot remove the first vertex. Check if the second vertex can be removed.
                  if (!isVertexPreventingKink(secondEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the second vertex.
                     concaveHullVerticesToFilter.remove(secondEdgeVertexIndex);
                     nVerticesRemoved++;
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
                  nVerticesRemoved++;
               }
               else
               { // Cannot remove the second vertex. Check if the first vertex can be removed.
                  if (!isVertexPreventingKink(firstEdgeVertexIndex, concaveHullVerticesToFilter))
                  { // Can remove the first vertex.
                     concaveHullVerticesToFilter.remove(firstEdgeVertexIndex);
                     nVerticesRemoved++;
                  }
                  else
                  { // Cannot removed the first vertex either, skip.
                     beforeEdgeVertexIndex++;
                  }
               }
            }
         }
      }
      return nVerticesRemoved;
   }
}
