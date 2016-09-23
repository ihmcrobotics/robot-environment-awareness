package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.geometry.GeometryTools.isPointInsideTriangleABC;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
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

   public static boolean areLineSegmentsIntersecting(Point2d firstSegment0, Point2d firstSegment1, Point2d secondSegment0, Point2d secondSegment1)
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

   public static boolean computeConcaveHullPocket(int concaveVertexIndex, ConcaveHullPocket pocketToPack, List<Point2d> concaveHullVertices)
   {
      pocketToPack.clear();
      boolean success = findBridgeVertices(concaveVertexIndex, pocketToPack, concaveHullVertices);
      if (!success)
      {
         pocketToPack.clear();
         return false;
      }
      success = findDeepestVertexInPocket(pocketToPack, concaveHullVertices);
      if (!success)
      {
         pocketToPack.clear();
         return false;
      }
      return true;
   }

   public static ConcaveHullPocket computeConcaveHullPocket(int concaveVertexIndex, List<Point2d> concaveHullVertices)
   {
      ConcaveHullPocket pocketToReturn = new ConcaveHullPocket();
      boolean success = findBridgeVertices(concaveVertexIndex, pocketToReturn, concaveHullVertices);
      if (!success)
         return null;
      success = findDeepestVertexInPocket(pocketToReturn, concaveHullVertices);
      if (!success)
         return null;
      return pocketToReturn;
   }

   /**
    * Find the two vertices that forms a bridge over the pocket that the concaveVertex belongs to.
    * @param concaveVertexIndex
    * @param pocketToPack
    * @param concaveHullVertices
    * @return {firstBridgeIndex, secondBridgeIndex} or null if the polygon is actually convex at the given vertex.
    */
   public static boolean findBridgeVertices(int concaveVertexIndex, ConcaveHullPocket pocketToPack, List<Point2d> concaveHullVertices)
   {
      concaveVertexIndex %= concaveHullVertices.size();
      Point2d concaveVertex = concaveHullVertices.get(concaveVertexIndex);

      int firstBridgeIndex = decrement(concaveVertexIndex, concaveHullVertices);
      int secondBridgeIndex = increment(concaveVertexIndex, concaveHullVertices);

      Point2d firstBridgeVertex = concaveHullVertices.get(firstBridgeIndex);
      Point2d secondBridgeVertex = concaveHullVertices.get(secondBridgeIndex);

      // The polygon is convex at this vertex => no pocket => no bridge
      if (GeometryTools.isPointOnLeftSideOfLine(concaveVertex, firstBridgeVertex, secondBridgeVertex))
         return false;

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
            boolean isBridgeGoingThroughPolygon = false;

            for (int i = increment(startIndexCandidate, concaveHullVertices); i != decrement(secondBridgeIndex, concaveHullVertices); i = increment(i, concaveHullVertices))
            {
               Point2d vertex = concaveHullVertices.get(i);
               Point2d nextVertex = concaveHullVertices.get(increment(i, concaveHullVertices));
               if (areLineSegmentsIntersecting(startCandidate, secondBridgeVertex, vertex, nextVertex))
               {
                  isBridgeGoingThroughPolygon = true;
                  break;
               }
            }

            if (!isBridgeGoingThroughPolygon)
            {
               firstBridgeIndex = startIndexCandidate;
               firstBridgeVertex = startCandidate;
               // Reset the other index to rescan the vertices
               endIndexCandidate = secondBridgeIndex;
            }
         }
         else if (GeometryTools.isPointOnLeftSideOfLine(endCandidate, firstBridgeVertex, secondBridgeVertex))
         { // endIndexCandidate is the new secondBridgeIndex.
            boolean isBridgeGoingThroughPolygon = false;

            for (int i = increment(firstBridgeIndex, concaveHullVertices); i != decrement(endIndexCandidate, concaveHullVertices); i = increment(i, concaveHullVertices))
            {
               Point2d vertex = concaveHullVertices.get(i);
               Point2d nextVertex = concaveHullVertices.get(increment(i, concaveHullVertices));
               if (areLineSegmentsIntersecting(firstBridgeVertex, endCandidate, vertex, nextVertex))
               {
                  isBridgeGoingThroughPolygon = true;
                  break;
               }
            }

            if (!isBridgeGoingThroughPolygon)
            {
               secondBridgeIndex = endIndexCandidate;
               secondBridgeVertex = endCandidate;
               // Reset the other index to rescan the vertices
               startIndexCandidate = firstBridgeIndex;
            }
         }
      }

      pocketToPack.setBridgeIndices(firstBridgeIndex, secondBridgeIndex);
      pocketToPack.setBridgeVertices(firstBridgeVertex, secondBridgeVertex);

      return true;
   }

   public static ConcaveHullPocket findFirstConcaveHullPocket(List<Point2d> concaveHullVertices)
   {
      ConvexPolygon2d convexHull = new ConvexPolygon2d(concaveHullVertices);

      // Find first common vertex between the two hulls. 
      int convexStartIndex = 0;
      int concaveStartIndex = -1;

      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point2d currentConvexVertex = convexHull.getVertex(i);

         for (int j = 0; j < concaveHullVertices.size(); j++)
         {
            Point2d currentConcaveVertex = concaveHullVertices.get(j);

            if (currentConcaveVertex.epsilonEquals(currentConvexVertex, 1.0e-7))
            {
               concaveStartIndex = j;
               break;
            }
         }

         if (convexStartIndex != -1)
            break;
      }

      if (convexStartIndex == -1 || concaveStartIndex == -1)
         return null;

      // Find the first index at which a bridge starts
      int startBridgeConcaveIndex = -1;
      int startBridgeConvexIndex = -1;

      for (int indexOffset = 1; indexOffset < concaveHullVertices.size(); indexOffset++)
      {
         int currentConvexIndex = (convexStartIndex + indexOffset) % convexHull.getNumberOfVertices();
         int currentConcaveIndex = (concaveStartIndex + indexOffset) % concaveHullVertices.size();
         Point2d currentConvexVertex = convexHull.getVertex(currentConvexIndex);
         Point2d currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

         if (!currentConvexVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
         {
            startBridgeConvexIndex = currentConvexIndex - 1;
            if (startBridgeConvexIndex == -1)
               startBridgeConvexIndex = convexHull.getNumberOfVertices() - 1;
            startBridgeConcaveIndex = currentConcaveIndex - 1;
            if (startBridgeConcaveIndex == -1)
               startBridgeConcaveIndex = concaveHullVertices.size() - 1;
            break;
         }
      }

      if (startBridgeConvexIndex == -1 || startBridgeConcaveIndex == -1)
         return null;

      // Find the deepest vertex in the pocket
      int endBridgeConcaveIndex = -1;

      Point2d firstBridgeVertex = convexHull.getVertex(startBridgeConvexIndex);
      Point2d secondBridgeVertex = convexHull.getNextVertex(startBridgeConvexIndex);
      LineSegment2d bridgeSegment = new LineSegment2d(firstBridgeVertex, secondBridgeVertex);

      int currentConcaveIndex = (startBridgeConcaveIndex + 1) % concaveHullVertices.size();
      Point2d currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

      int deepestPocketVertexIndex = -1;
      double pocketMaxDepth = 0.0;

      while (!secondBridgeVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
      {
         double currentDepth = bridgeSegment.distance(currentConcaveVertex);

         if (currentDepth > pocketMaxDepth)
         {
            deepestPocketVertexIndex = currentConcaveIndex;
            pocketMaxDepth = currentDepth;
         }

         currentConcaveIndex = (currentConcaveIndex + 1) % concaveHullVertices.size();
         currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);
      }

      endBridgeConcaveIndex = currentConcaveIndex;

      ConcaveHullPocket pocket = new ConcaveHullPocket();
      pocket.setBridgeIndices(startBridgeConcaveIndex, endBridgeConcaveIndex);
      pocket.setBridgeVertices(firstBridgeVertex, secondBridgeVertex);
      pocket.setMaxDepth(pocketMaxDepth);
      pocket.setDeepestVertex(concaveHullVertices.get(deepestPocketVertexIndex));
      pocket.setDeepestVertexIndex(deepestPocketVertexIndex);

      return pocket;
   }

   /**
    * Find the deepest vertex in the given pocket.
    * It updates {@link ConcaveHullPocket} fields accordingly.
    * The bridge indices are required to call this method.
    * @param pocketToModify
    * @param concaveHullVertices
    * @return
    */
   public static boolean findDeepestVertexInPocket(ConcaveHullPocket pocketToModify, List<Point2d> concaveHullVertices)
   {
      pocketToModify.clearDepthParameters();

      int startBridgeIndex = pocketToModify.getStartBridgeIndex();
      int endBridgeIndex = pocketToModify.getEndBridgeIndex();

      Point2d startBridgeVertex = concaveHullVertices.get(startBridgeIndex);
      Point2d endBridgeVertex = concaveHullVertices.get(endBridgeIndex);

      for (int index = increment(startBridgeIndex, concaveHullVertices); index != endBridgeIndex; index = increment(index, concaveHullVertices))
      {
         Point2d vertex = concaveHullVertices.get(index);
         double depth = GeometryTools.distanceFromPointToLine(vertex, startBridgeVertex, endBridgeVertex);

         if (depth > pocketToModify.getMaxDepth())
         {
            pocketToModify.setDeepestVertexIndex(index);
            pocketToModify.setDeepestVertex(vertex);
            pocketToModify.setMaxDepth(depth);
         }
      }

      return pocketToModify.getDeepestVertexIndex() >= 0;
   }

   public static boolean isConvexAtVertex(int vertexIndex, List<Point2d> concaveHullVertices)
   {
      Point2d vertex = concaveHullVertices.get(vertexIndex);
      Point2d previousVertex = concaveHullVertices.get(decrement(vertexIndex, concaveHullVertices));
      Point2d nextVertex = concaveHullVertices.get(increment(vertexIndex, concaveHullVertices));

      return GeometryTools.isPointOnLeftSideOfLine(vertex, previousVertex, nextVertex);
   }

   public static boolean isAlmostConvexAtVertex(int vertexIndex, double angleTolerance, List<Point2d> concaveHullVertices)
   {
      Point2d vertex = concaveHullVertices.get(vertexIndex);
      Point2d previousVertex = concaveHullVertices.get(decrement(vertexIndex, concaveHullVertices));
      Point2d nextVertex = concaveHullVertices.get(increment(vertexIndex, concaveHullVertices));

      return getAngleABC(nextVertex, previousVertex, vertex) > -angleTolerance;
   }

   public static double getAngleABC(Point2d a, Point2d b, Point2d c)
   {
      double bax = b.getX() - a.getX();
      double bay = b.getY() - a.getY();

      double bcx = b.getX() - c.getX();
      double bcy = b.getY() - c.getY();

      return GeometryTools.getAngleFromFirstToSecondVector(bax, bay, bcx, bcy);
   }

   public static double getAngleFromPreviousEdgeToNextEdge(int vertexIndex, List<Point2d> concaveHullVertices)
   {
      Point2d vertex = concaveHullVertices.get(vertexIndex);
      Point2d previousVertex = concaveHullVertices.get(decrement(vertexIndex, concaveHullVertices));
      Point2d nextVertex = concaveHullVertices.get(increment(vertexIndex, concaveHullVertices));

      double previousEdgeX = vertex.getX() - previousVertex.getX();
      double previousEdgeY = vertex.getY() - previousVertex.getY();
      double nextEdgeX = nextVertex.getX() - vertex.getX();
      double nextEdgeY = nextVertex.getY() - vertex.getY();
      return GeometryTools.getAngleFromFirstToSecondVector(previousEdgeX, previousEdgeY, nextEdgeX, nextEdgeY);
   }

   public static int increment(int index, List<Point2d> concaveHullVertices)
   {
      return (index + 1) % concaveHullVertices.size();
   }

   public static int decrement(int index, List<Point2d> concaveHullVertices)
   {
      return (index - 1 + concaveHullVertices.size()) % concaveHullVertices.size();
   }

   public static Point2d getWrap(List<Point2d> list, int index)
   {
      if (index == -1)
         return list.get(list.size() - 1);
      return list.get(index % list.size());
   }

   public static List<Point2d> subList(List<Point2d> input, int startIndex, int endIndex)
   {
      List<Point2d> output = new ArrayList<>();

      int outputLenth;
      if (endIndex > startIndex)
         outputLenth = endIndex - startIndex;
      else
         outputLenth = (endIndex + input.size()) - startIndex;

      int i = startIndex;
      while (output.size() != outputLenth)
      {
         output.add(getWrap(input, i));
         i++;
      }

      return output;
   }

   public static int removeAllBetween(int fromIndex, int toIndex, List<Point2d> list)
   {
      int numberOfElementsToRemove = (toIndex - fromIndex + list.size() - 1) % list.size();
      
      for (int count = 0; count < numberOfElementsToRemove; count++)
      {
         int firstRemovableIndex = increment(fromIndex, list);
         list.remove(firstRemovableIndex);
      }
      return numberOfElementsToRemove;
   }

   public static boolean isHullConvex(List<Point2d> concaveHullVertices)
   {
      if (concaveHullVertices.size() <= 3)
         return true;

      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         if (!isConvexAtVertex(i, concaveHullVertices))
            return false;
      }

      return true;
   }
}
