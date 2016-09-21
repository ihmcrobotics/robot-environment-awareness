package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.List;

import javax.vecmath.Point2d;

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
      int vertexPreviousIndex = (vertexIndex - 1 + concaveHullVertices.size()) % concaveHullVertices.size();
      vertexIndex %= concaveHullVertices.size();
      int vertexNextIndex = (vertexIndex + 1) % concaveHullVertices.size();
   
      Point2d a = concaveHullVertices.get(vertexPreviousIndex);
      Point2d b = concaveHullVertices.get(vertexIndex);
      Point2d c = concaveHullVertices.get(vertexNextIndex);
   
      int currentIndex = (vertexNextIndex + 1) % concaveHullVertices.size();
   
      while (currentIndex != vertexPreviousIndex)
      {
         if (GeometryTools.isPointInsideTriangleABC(concaveHullVertices.get(currentIndex), a, b, c))
            return true;
         currentIndex = (currentIndex +1) % concaveHullVertices.size();
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

}
