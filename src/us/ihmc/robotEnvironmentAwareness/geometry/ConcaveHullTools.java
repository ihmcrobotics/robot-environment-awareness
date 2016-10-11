package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ListTools.*;
import static us.ihmc.robotics.geometry.GeometryTools.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.Line2d;
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

   public static void ensureCounterClockwiseOrdering(List<Point2d> concaveHullVertices)
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

      if (sumOfAngles < 0.0)
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

   public static ConcaveHullPocket findFirstConcaveHullPocket(List<Point2d> concaveHullVertices)
   {
      ConcaveHullPocket pocket = null;

      for (int i = 0; i < concaveHullVertices.size() && pocket == null; i++)
         pocket = ConcaveHullTools.computeConcaveHullPocket(i, concaveHullVertices);

      return pocket;
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
         { // startIndexCandidate is a potential firstBridgeIndex.
            boolean isBridgeCoveringPocket = false;

            // Make sure that the new bridge would go over all the pocket vertices
            for (int i = increment(startIndexCandidate, concaveHullVertices); i != secondBridgeIndex
                  && isBridgeCoveringPocket; i = increment(i, concaveHullVertices))
               isBridgeCoveringPocket = !isPointOnLeftSideOfLine(concaveHullVertices.get(i), startCandidate, secondBridgeVertex);

            if (isBridgeCoveringPocket)
            {
               firstBridgeIndex = startIndexCandidate;
               firstBridgeVertex = startCandidate;
               // Reset the other index to rescan the vertices
               endIndexCandidate = secondBridgeIndex;
            }
         }
         else if (isPointOnLeftSideOfLine(endCandidate, firstBridgeVertex, secondBridgeVertex))
         { // endIndexCandidate is the new secondBridgeIndex.
            boolean isBridgeCoveringPocket = true;

            // Make sure that the new bridge would go over all the pocket vertices
            for (int i = increment(firstBridgeIndex, concaveHullVertices); i != endIndexCandidate
                  && isBridgeCoveringPocket; i = increment(i, concaveHullVertices))
               isBridgeCoveringPocket = !isPointOnLeftSideOfLine(concaveHullVertices.get(i), firstBridgeVertex, endCandidate);

            if (isBridgeCoveringPocket)
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
         double depth = distanceFromPointToLine(vertex, startBridgeVertex, endBridgeVertex);

         if (depth > pocketToModify.getMaxDepth())
         {
            pocketToModify.setDeepestVertexIndex(index);
            pocketToModify.setDeepestVertex(vertex);
            pocketToModify.setMaxDepth(depth);
         }
      }

      return pocketToModify.getDeepestVertexIndex() >= 0;
   }

   public static ConcaveHullPocket findFirstConcaveHullPocketInefficient(List<Point2d> concaveHullVertices)
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
      Point2d previousVertex = getPreviousWrap(vertexIndex, concaveHullVertices);
      Point2d nextVertex = getNextWrap(vertexIndex, concaveHullVertices);

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
      Point2d previousVertex = getPreviousWrap(vertexIndex, concaveHullVertices);
      Point2d nextVertex = getNextWrap(vertexIndex, concaveHullVertices);

      double previousEdgeX = vertex.getX() - previousVertex.getX();
      double previousEdgeY = vertex.getY() - previousVertex.getY();
      double nextEdgeX = nextVertex.getX() - vertex.getX();
      double nextEdgeY = nextVertex.getY() - vertex.getY();
      return GeometryTools.getAngleFromFirstToSecondVector(previousEdgeX, previousEdgeY, nextEdgeX, nextEdgeY);
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

   /**
    * Find the closest edge to a vertex that is visible from the inside of the polygon.
    * In other words, the line that goes from the given vertex to the computed closestPoint is entirely inside the polygon.
    * @param vertexIndex the close edge to this vertex.
    * @param deadIndexRegion the search algorithm starts at {@code vertexIndex + deadIndexRegion + 1} and ends at {@code vertexIndex - deadIndexRegion - 1}
    * @param concaveHullVertices the vertices of the concave hull on which the search is to be performed.
    * @param closestPointToPack coordinates of the closest point found.
    * @return the index of the closest edge first vertex.
    */
   public static int findInnerClosestEdgeToVertex(int vertexIndex, int deadIndexRegion, List<Point2d> concaveHullVertices, Point2d closestPointToPack)
   {
      int startSearchIndex = increment(vertexIndex, concaveHullVertices);
      int endSearchIndex = decrement(vertexIndex, concaveHullVertices);
      return findInnerClosestEdgeToVertex(vertexIndex, startSearchIndex, endSearchIndex, concaveHullVertices, closestPointToPack);
   }

   public static int findInnerClosestEdgeToVertex(int vertexIndex, int startSearchIndex, int endSearchIndex, List<Point2d> concaveHullVertices, Point2d closestPointToPack)
   {
      int closestEdgeFirstIndex = -1;
      double distanceSquaredToClosestEdge = Double.POSITIVE_INFINITY;

      vertexIndex %= concaveHullVertices.size();
      Point2d vertex = concaveHullVertices.get(vertexIndex);

      LineSegment2d edge = new LineSegment2d();
      Point2d candidateClosestPoint = new Point2d();

      // The loop skips the edges to which the given vertex belongs.
      for (int candidateIndex = startSearchIndex; candidateIndex != endSearchIndex; candidateIndex = increment(candidateIndex, concaveHullVertices))
      {
         Point2d edgeFirstVertex = concaveHullVertices.get(candidateIndex);
         Point2d edgeSecondVertex = getNextWrap(candidateIndex, concaveHullVertices);

         edge.set(edgeFirstVertex, edgeSecondVertex);
         edge.getClosestPointOnLineSegment(candidateClosestPoint, vertex);

         double distanceSquared = candidateClosestPoint.distanceSquared(vertex);

         // We have a new point that is closer than the previous.
         if (distanceSquared < distanceSquaredToClosestEdge)
         {
            // Before considering it as the new closest point, we need to check that the line goes from the given vertex to the new candidate is fully inside of the polygon.
            boolean isLineInsidePolygon = true;

            int startCheckIndex = increment(startSearchIndex, concaveHullVertices);
            int endCheckIndex = increment(candidateIndex, concaveHullVertices);

            // The line is inside the polygon if all the vertices in ]vertexIndex; candidateIndex[ are on the left side of the line (vertex, candidateClosestPoint)
            for (int index = startCheckIndex; index != endCheckIndex && isLineInsidePolygon; index = increment(index, concaveHullVertices))
               isLineInsidePolygon = isPointOnLeftSideOfLine(concaveHullVertices.get(index), vertex, candidateClosestPoint);

            if (isLineInsidePolygon)
            { // The line is inside, the candidate is the new closest point.
               distanceSquaredToClosestEdge = distanceSquared;
               closestEdgeFirstIndex = candidateIndex;
               closestPointToPack.set(candidateClosestPoint);
            }
         }
      }

      return closestEdgeFirstIndex;
   }

   public static int findClosestIntersectionWithRay(Point2d rayOrigin, Vector2d rayDirection, int startSearchIndex, int endSearchIndex, List<Point2d> concaveHullVertices, Point2d intersectionToPack)
   {
      double minDistanceSquared = Double.POSITIVE_INFINITY;
      int closestEdgeFirstVertexIndex = -1;
      intersectionToPack.set(Double.NaN, Double.NaN);

      Vector2d rayOriginToCandidate = new Vector2d();
      Line2d rayLine = new Line2d(rayOrigin, rayDirection);
      LineSegment2d edge = new LineSegment2d();

      for (int currentIndex = startSearchIndex; currentIndex != endSearchIndex; currentIndex = increment(currentIndex, concaveHullVertices))
      {
         int nextIndex = increment(currentIndex, concaveHullVertices);

         Point2d current = concaveHullVertices.get(currentIndex);
         Point2d next = concaveHullVertices.get(nextIndex);

         edge.set(current, next);
         Point2d intersection = edge.intersectionWith(rayLine);

         if (intersection != null)
         {
            rayOriginToCandidate.sub(intersection, rayOrigin);
            if (rayOriginToCandidate.dot(rayDirection) < 0.0)
               continue;
            
            double distanceSquared = intersection.distanceSquared(rayOrigin);
            if (distanceSquared < minDistanceSquared)
            {
               minDistanceSquared = distanceSquared;
               intersectionToPack.set(intersection);
               closestEdgeFirstVertexIndex = currentIndex;
            }
         }
      }
      return closestEdgeFirstVertexIndex;
   }

   public static String vertexListToString(List<Point2d> vertexList)
   {
      String ret = "";
      for (int i = 0; i < vertexList.size(); i++)
      {
         Point2d vertex = vertexList.get(i);
         ret += vertex.getX() + ", " + vertex.getY();
         if (i < vertexList.size() - 1)
            ret += "\n";
      }
      return ret;
   }

   public static void exportVertexListToFile(List<Point2d> vertexList, String fileName)
   {
      try
      {
         File file = new File(fileName);
         if(!file.exists())
            file.createNewFile();
         FileWriter fileWriter = new FileWriter(file);
         fileWriter.write(vertexListToString(vertexList));
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
