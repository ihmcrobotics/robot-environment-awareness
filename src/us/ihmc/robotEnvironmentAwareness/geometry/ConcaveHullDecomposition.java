package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.*;

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
         throw new RuntimeException("The concave hull is empty");

      // The concave hull is actually convex, end of recursion
      if (ConcaveHullTools.isHullConvex(concaveHullVertices))
      {
         convexPolygons.add(new ConvexPolygon2d(concaveHullVertices));
         return;
      }

      ConcaveHullPocket pocket = null;
      if (concaveHullVertices.size() == 45)
         System.out.println();
      // Find the first pocket
      for (int i = 0; i < concaveHullVertices.size() && pocket == null; i++)
      {
         if (!isConvexAtVertex(i, concaveHullVertices))
            pocket = ConcaveHullTools.computeConcaveHullPocket(i, concaveHullVertices);
      }
      ConcaveHullPocket pocket2 = ConcaveHullTools.findFirstConcaveHullPocket(concaveHullVertices);

      if (pocket2.getStartBridgeIndex() != pocket.getStartBridgeIndex())
         System.out.println();
      if (pocket2.getEndBridgeIndex() != pocket.getEndBridgeIndex())
         System.out.println();

//      pocket = pocket2;

      if (pocket == null)
         throw new RuntimeException("Did not find any pocket.");

      int startBridgeIndex = pocket.getStartBridgeIndex();
      int endBridgeIndex = pocket.getEndBridgeIndex();

      // The pocket is negligible, remove the vertices.
      if (pocket.getMaxDepth() < depthThreshold)
      {
         ConcaveHullTools.removeAllBetween(startBridgeIndex, endBridgeIndex, concaveHullVertices);
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
//      cutDirection.sub(concaveHullVertices.get(increment(deepestVertexIndex, concaveHullVertices)), concaveHullVertices.get(decrement(deepestVertexIndex, concaveHullVertices)));
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
         System.err.println("Something went wrong finding the other vertex for cutting. Pocket vertex: " + deepestVertex + ", bridge: start: " + startBridgeVertex + ", end: " + endBridgeVertex);
         return;
      }

      if (alpha < 1.0e-3)
      {
         otherVertexIndexForCutting = (otherVertexIndexForCutting - 1) % concaveHullVertices.size();
      }
      else if (alpha < 1.0 - 1.0e-3)
      {
         concaveHullVertices.add(otherVertexIndexForCutting, otherVertexForCutting);
         if (otherVertexIndexForCutting < deepestVertexIndex)
            deepestVertexIndex++;
      }

      // decompose the two new polygons.
      int p1StartIndex = deepestVertexIndex;
      int p1EndIndex = (otherVertexIndexForCutting + 1) % (concaveHullVertices.size() + 1);
      int p2StartIndex = otherVertexIndexForCutting;
      int p2EndIndex = (deepestVertexIndex + 1) % (concaveHullVertices.size() + 1);

      int p1Size;
      if (p1StartIndex > p1EndIndex)
         p1Size = p1EndIndex + concaveHullVertices.size() - p1StartIndex;
      else
         p1Size = p1EndIndex - p1StartIndex;
      int p2Size;
      if (p2StartIndex > p2EndIndex)
         p2Size = p2EndIndex + concaveHullVertices.size() - p2StartIndex;
      else
         p2Size = p2EndIndex - p2StartIndex;

      if (p1Size == concaveHullVertices.size() || p2Size == concaveHullVertices.size())
         throw new RuntimeException("Something went wrong.");

      
      List<Point2d> p1 = subList(concaveHullVertices, p1StartIndex, p1EndIndex);
      List<Point2d> p2 = subList(concaveHullVertices, p2StartIndex, p2EndIndex);
      if (p1.isEmpty() || p2.isEmpty())
         System.out.println();
      decomposeRecursively(p1, depthThreshold, decompositionDepth + 1, convexPolygons);
      decomposeRecursively(p2, depthThreshold, decompositionDepth + 1, convexPolygons);
   }

}
