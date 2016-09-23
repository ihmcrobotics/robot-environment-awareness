package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.increment;
import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.removeAllExclusive;
import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.subListInclusive;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;

public class ConcaveHullDecomposition
{

   public static void decomposeRecursively(List<Point2d> concaveHullVertices, double depthThreshold, int decompositionDepth,
         List<ConvexPolygon2d> convexPolygons)
   {
      if (concaveHullVertices.isEmpty())
      {
         System.err.println("The concave hull is empty");
         return;
      }

      // The concave hull is actually convex, end of recursion
      if (ConcaveHullTools.isHullConvex(concaveHullVertices))
      {
         convexPolygons.add(new ConvexPolygon2d(concaveHullVertices));
         return;
      }

      ConcaveHullPocket pocket = ConcaveHullTools.findFirstConcaveHullPocket(concaveHullVertices);

      if (pocket == null)
      {
         System.err.println("Did not find any pocket.");
         return;
      }

      int startBridgeIndex = pocket.getStartBridgeIndex();
      int endBridgeIndex = pocket.getEndBridgeIndex();

      // The pocket is negligible, remove the vertices.
      if (pocket.getMaxDepth() < depthThreshold)
      {
         removeAllExclusive(startBridgeIndex, endBridgeIndex, concaveHullVertices);
         // Restart the search for pockets
         decomposeRecursively(concaveHullVertices, depthThreshold, decompositionDepth, convexPolygons);
         return;
      }

      if (startBridgeIndex == 36 && endBridgeIndex == 16)
         System.out.println();

      int deepestVertexIndex = pocket.getDeepestVertexIndex();
      Vector2d cutDirection = new Vector2d();
      Point2d endBridgeVertex = pocket.getEndBridgeVertex();
      Point2d startBridgeVertex = pocket.getStartBridgeVertex();
      cutDirection.sub(endBridgeVertex, startBridgeVertex);
      cutDirection.normalize();
      cutDirection.set(cutDirection.y, -cutDirection.x); // Rotate 90 degrees to the right (inside polygon)

      Point2d deepestVertex = pocket.getDeepestVertex();
      Line2d cuttingLine = new Line2d(deepestVertex, cutDirection);
      LineSegment2d edge = new LineSegment2d();

      int otherVertexIndexForCutting = -1;
      Point2d otherVertexForCutting = new Point2d(Double.NaN, Double.NaN);
      double distanceSquaredFromOtherVertex = Double.POSITIVE_INFINITY;
      double alpha = Double.NaN;

      for (int currentIndex = endBridgeIndex; currentIndex != startBridgeIndex; currentIndex = increment(currentIndex, concaveHullVertices))
      {
         int nextIndex = increment(currentIndex, concaveHullVertices);

         Point2d current = concaveHullVertices.get(currentIndex);
         Point2d next = concaveHullVertices.get(nextIndex);

         edge.set(current, next);
         Point2d intersection = edge.intersectionWith(cuttingLine);
         if (intersection != null)
         {
            double distanceSquared = intersection.distanceSquared(deepestVertex);
            if (distanceSquared < distanceSquaredFromOtherVertex)
            {
               distanceSquaredFromOtherVertex = distanceSquared;
               otherVertexForCutting.set(intersection);
               otherVertexIndexForCutting = nextIndex;
               alpha = edge.percentageAlongLineSegment(intersection);
            }
         }
      }

      if (Double.isNaN(otherVertexForCutting.getX()))
      {
         System.err.println("Something went wrong finding the other vertex for cutting. Pocket vertex: " + deepestVertex + ", bridge: start: "
               + startBridgeVertex + ", end: " + endBridgeVertex);
         return;
      }

      // Check if we can use an existing vertex to cut
      if (alpha < 0.1)
      {
         otherVertexIndexForCutting = (otherVertexIndexForCutting - 1) % concaveHullVertices.size();
      }
      else if (alpha > 1.0 - 0.1)
      {
         // otherVertexIndexForCutting is good
      }
      else
      { // Add a new vertex
         concaveHullVertices.add(otherVertexIndexForCutting, otherVertexForCutting);
         if (otherVertexIndexForCutting < deepestVertexIndex)
            deepestVertexIndex++;
      }

      // decompose the two new polygons.
      int p1StartIndex = deepestVertexIndex;
      int p1EndIndex = otherVertexIndexForCutting;
      int p2StartIndex = otherVertexIndexForCutting;
      int p2EndIndex = deepestVertexIndex;

      List<Point2d> p1 = subListInclusive(p1StartIndex, p1EndIndex, concaveHullVertices);
      List<Point2d> p2 = subListInclusive(p2StartIndex, p2EndIndex, concaveHullVertices);

      decomposeRecursively(p1, depthThreshold, decompositionDepth + 1, convexPolygons);
      decomposeRecursively(p2, depthThreshold, decompositionDepth + 1, convexPolygons);
   }

}
