package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.geometry.GeometryTools.isPointInsideTriangleABC;

import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.LineSegment2d;

public class ConcaveHullTools
{
   /**
    * Returns true only if removing the vertex would generate a kink in the concave polygon.
    * Meaning, it would cause several edges to cross each other.
    */
   public static boolean isVertexPreventingKink(int vertexIndex, List<Point2d> concaveHullVertices)
   {
      int vertexPreviousIndex = decrement(vertexIndex, concaveHullVertices);
      vertexIndex %= concaveHullVertices.size();
      int vertexNextIndex = increment(vertexIndex, concaveHullVertices);
   
      Point2d a = concaveHullVertices.get(vertexPreviousIndex);
      Point2d b = concaveHullVertices.get(vertexIndex);
      Point2d c = concaveHullVertices.get(vertexNextIndex);
   
      int currentIndex = increment(vertexNextIndex, concaveHullVertices);
   
      while (currentIndex != vertexPreviousIndex)
      {
         if (isPointInsideTriangleABC(concaveHullVertices.get(currentIndex), a, b, c))
            return true;
         currentIndex = increment(currentIndex, concaveHullVertices);
      }
   
      return false;
   }

   public static Point2d intersectionFromEndPoints(Point2d firstSegment0, Point2d firstSegment1, Point2d secondSegment0, Point2d secondSegment1)
   {
      LineSegment2d first = new LineSegment2d(firstSegment0, firstSegment1);
      LineSegment2d second = new LineSegment2d(secondSegment0, secondSegment1);
   
      Point2d ret = first.intersectionWith(second);
      if (ret == null)
         return null;
      else
         return new Point2d(ret);
   }

   public static boolean areLineSegmentsIntersection(Point2d firstSegment0, Point2d firstSegment1, Point2d secondSegment0, Point2d secondSegment1)
   {
      LineSegment2d first = new LineSegment2d(firstSegment0, firstSegment1);
      LineSegment2d second = new LineSegment2d(secondSegment0, secondSegment1);
      return first.intersectionWith(second) != null;
   }

   public static void ensureClockwiseOrdering(List<Point2d> concaveHullVertices)
   {
      double sumOfAngles = 0.0;

      Vector2d previousEdge = new Vector2d();
      Vector2d nextEdge = new Vector2d();

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.size(); vertexIndex++)
      {
         int previousVertexIndex = decrement(vertexIndex, concaveHullVertices);
         int nextVertexIndex = increment(vertexIndex, concaveHullVertices);

         Point2d previousVertex = concaveHullVertices.get(previousVertexIndex);
         Point2d vertex = concaveHullVertices.get(vertexIndex);
         Point2d nextVertex = concaveHullVertices.get(nextVertexIndex);

         previousEdge.sub(vertex, previousVertex);
         nextEdge.sub(nextVertex, vertex);
         sumOfAngles += GeometryTools.getAngleFromFirstToSecondVector(previousEdge, nextEdge);
      }

      if (sumOfAngles > 0.0)
         Collections.reverse(concaveHullVertices);
   }

   public static double computePerimeter(List<Point2d> concaveHullVertices)
   {
      double perimeter = 0.0;
      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Point2d vertex = concaveHullVertices.get(i);
         Point2d nextVertex = concaveHullVertices.get(increment(i, concaveHullVertices));
         perimeter += vertex.distance(nextVertex);
      }
      return perimeter;
   }

   public static int removeSuccessiveDuplicateVertices(List<Point2d> concaveHullVertices)
   {
      int numberOfVerticesRemoved = 0;

      for (int index = 0; index < concaveHullVertices.size();)
      {
         int nextIndex = increment(index, concaveHullVertices);
         if (concaveHullVertices.get(index).equals(concaveHullVertices.get(nextIndex)))
         {
            concaveHullVertices.remove(nextIndex);
            numberOfVerticesRemoved++;
         }
         else
         {
            index++;
         }
      }
      return numberOfVerticesRemoved;
   }

   /**
    * Find the two vertices that forms a bridge over the pocket that the concaveVertex belongs to.
    * @param concaveVertexIndex
    * @param concaveHullVertices
    * @return {firstBridgeIndex, secondBridgeIndex} or null if the polygon is actually convex at the given vertex.
    */
   public static int[] findBridgeIndices(int concaveVertexIndex, List<Point2d> concaveHullVertices)
   {
      concaveVertexIndex %= concaveHullVertices.size();
      Point2d concaveVertex = concaveHullVertices.get(concaveVertexIndex);

      int firstBridgeIndex = decrement(concaveVertexIndex, concaveHullVertices);
      int secondBridgeIndex = increment(concaveVertexIndex, concaveHullVertices);

      Point2d firstBridgeVertex = concaveHullVertices.get(firstBridgeIndex);
      Point2d secondBridgeVertex = concaveHullVertices.get(secondBridgeIndex);

      // The polygon is convex at this vertex => no pocket => no bridge
      if (GeometryTools.isPointOnLeftSideOfLine(concaveVertex, firstBridgeVertex, secondBridgeVertex))
         return null;

      int startIndexCandidate = firstBridgeIndex;
      int endIndexCandidate = secondBridgeIndex;

      // Doing a loop with the start and end indices going opposite ways at the same time.
      while (true)
      {
         startIndexCandidate = decrement(startIndexCandidate, concaveHullVertices);
         // Check if all the vertices have been explored
         if (startIndexCandidate == endIndexCandidate)
            break;

         endIndexCandidate = increment(endIndexCandidate, concaveHullVertices);
         // Check if all the vertices have been explored
         if (startIndexCandidate == endIndexCandidate)
            break;

         Point2d startCandidate = concaveHullVertices.get(startIndexCandidate);
         Point2d endCandidate = concaveHullVertices.get(endIndexCandidate);

         if (GeometryTools.isPointOnLeftSideOfLine(startCandidate, firstBridgeVertex, secondBridgeVertex))
         { // startIndexCandidate is the new firstBridgeIndex.
            firstBridgeIndex = startIndexCandidate;
            firstBridgeVertex = startCandidate;
            // Reset the other index to rescan the vertices
            endIndexCandidate = secondBridgeIndex;
         }
         else if (GeometryTools.isPointOnLeftSideOfLine(endCandidate, firstBridgeVertex, secondBridgeVertex))
         { // endIndexCandidate is the new secondBridgeIndex.
            secondBridgeIndex = endIndexCandidate;
            secondBridgeVertex = endCandidate;
            // Reset the other index to rescan the vertices
            startIndexCandidate = firstBridgeIndex;
         }
      }

      return new int[] {firstBridgeIndex, secondBridgeIndex};
   }

   public static int increment(int index, List<Point2d> concaveHullVertices)
   {
      return (index + 1) % concaveHullVertices.size();
   }

   public static int decrement(int index, List<Point2d> concaveHullVertices)
   {
      return (index - 1 + concaveHullVertices.size()) % concaveHullVertices.size();
   }
}
