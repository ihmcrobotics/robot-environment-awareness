package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.LineSegment1d;
import us.ihmc.robotics.geometry.LineSegment3d;

public class PlanarRegionIntersectionCalculator
{
   public static List<LineSegment3d> computeIntersections(List<PlanarRegionSegmentationRawData> rawData, IntersectionEstimationParameters parameters)
   {
      List<LineSegment3d> result = new ArrayList<>();

      Point3d intersectionPoint = new Point3d();
      Vector3d intersectionDirection = new Vector3d();

      for (int i = 0; i < rawData.size(); i++)
      {
         PlanarRegionSegmentationRawData currentRegion = rawData.get(i);

         if (currentRegion.size() < parameters.getMinRegionSize())
            continue;

         for (int j = i + 1; j < rawData.size(); j++)
         {
            PlanarRegionSegmentationRawData currentNeighbor = rawData.get(j);

            if (currentNeighbor.size() < parameters.getMinRegionSize())
               continue;

            boolean success = computeIntersectionPointAndDirection(currentRegion, currentNeighbor, intersectionPoint, intersectionDirection, parameters.getMinRegionAngleDifference());
            if (!success)
               continue;

            double maxDistanceToRegion = parameters.getMaxDistanceToRegion();
            double minIntersectionLength = parameters.getMinIntersectionLength();

            List<LineSegment3d> intersectionList = findIntersectionEndPoints2(currentRegion, currentNeighbor, maxDistanceToRegion, minIntersectionLength, intersectionPoint, intersectionDirection);

            if (intersectionList != null)
            {
               Vector3d intersectionLength = new Vector3d();

               result.addAll(intersectionList);

               if (parameters.isAddIntersectionsToRegions())
               {
                  for (LineSegment3d intersection : intersectionList)
                  {
                     intersectionLength.sub(intersection.getPointB(), intersection.getPointA());
                     double length = intersectionLength.length();
                     double delta = 0.01;
                     int numberOfPointsToAdd = (int) (length / delta);
                     for (int k = 0; k < numberOfPointsToAdd; k++)
                     {
                        Point3d newPoint = new Point3d();
                        double alpha = k / (double) numberOfPointsToAdd;
                        newPoint.scaleAdd(alpha, intersectionLength, intersection.getPointA());
                        // FIXME Figure out something less hackish
//                        currentRegion.addPoint(new Point3d(newPoint));
//                        currentNeighbor.addPoint(new Point3d(newPoint));
                     }
                  }
               }
            }
         }
      }

      return result;
   }

   private static boolean computeIntersectionPointAndDirection(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor, Point3d intersectionPointToPack,
         Vector3d intersectionDirectionToPack, double minRegionAngleDifference)
   {
      Point3d origin1 = currentRegion.getOrigin();
      Vector3d normal1 = currentRegion.getNormal();
      Point3d origin2 = currentNeighbor.getOrigin();
      Vector3d normal2 = currentNeighbor.getNormal();

      double angle = normal1.angle(normal2);

      if (MathTools.epsilonEquals(angle, 0.0, minRegionAngleDifference))// || MathTools.epsilonEquals(Math.abs(angle), Math.PI, epsilon))
         return false;

      intersectionDirectionToPack.cross(normal1, normal2);
      double det = intersectionDirectionToPack.lengthSquared();

      double d1 = normal1.dot(new Vector3d(origin1));
      double d2 = normal2.dot(new Vector3d(origin2));

      Vector3d normal3Cross2 = new Vector3d();
      normal3Cross2.cross(intersectionDirectionToPack, normal2);
      Vector3d normal1Cross3 = new Vector3d();
      normal1Cross3.cross(normal1, intersectionDirectionToPack);
      Vector3d normal2Cross1 = new Vector3d();
      normal2Cross1.cross(normal2, normal1);

      intersectionPointToPack.scale(d1, normal3Cross2);
      intersectionPointToPack.scaleAdd(d2, normal1Cross3, intersectionPointToPack);
      intersectionDirectionToPack.normalize();

      double d3 = 0.5 * (intersectionDirectionToPack.dot(new Vector3d(origin1)) + intersectionDirectionToPack.dot(new Vector3d(origin2)));
      intersectionPointToPack.scaleAdd(d3, normal2Cross1, intersectionPointToPack);

      intersectionPointToPack.scale(-1.0 / det);
      return true;
   }

   private static List<LineSegment3d> findIntersectionEndPoints2(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor, double maxDistance, double minIntersectionLength, Point3d intersectionPoint,
         Vector3d intersectionDirection)
   {

      List<LineSegment1d> intersectionsFromRegion1 = findIntersectionLineSegments(currentRegion, maxDistance, minIntersectionLength, intersectionPoint, intersectionDirection);
      if (intersectionsFromRegion1 == null || intersectionsFromRegion1.isEmpty())
         return null;
      List<LineSegment1d> intersectionsFromRegion2 = findIntersectionLineSegments(currentNeighbor, maxDistance, minIntersectionLength, intersectionPoint, intersectionDirection);
      if (intersectionsFromRegion2 == null || intersectionsFromRegion2.isEmpty())
         return null;

      List<LineSegment3d> intersections = new ArrayList<>();

      for (LineSegment1d intersectionFromRegion1 : intersectionsFromRegion1)
      {
         for (LineSegment1d intersectionFromRegion2 : intersectionsFromRegion2)
         {
            LineSegment1d overlap = intersectionFromRegion1.computeOverlap(intersectionFromRegion2);
            if (overlap != null && overlap.length() > minIntersectionLength)
               intersections.add(overlap.toLineSegment3d(intersectionPoint, intersectionDirection));
         }
      }

      return intersections.isEmpty() ? null : intersections;
   }

   /**
    * 
    * @param currentRegion
    * @param maxDistance
    * @param intersectionPoint
    * @param intersectionDirection
    * @return
    */
   private static List<LineSegment1d> findIntersectionLineSegments(PlanarRegionSegmentationRawData currentRegion, double maxDistance, double minIntersectionLength, Point3d intersectionPoint, Vector3d intersectionDirection)
   {
      Vector3d perpendicularToDirection = new Vector3d();
      perpendicularToDirection.cross(currentRegion.getNormal(), intersectionDirection);
      perpendicularToDirection.normalize();

      Vector3d distance = new Vector3d();
      Point3d regionPoint = new Point3d();

      // 1-D Coorsdinates along the intersection direction of all the region points that are close enough to the intersection.
      // By using a PriorityQueue the coordinates are sorted.
      PriorityQueue<Double> points1D = new PriorityQueue<>();

      for (int i = 0; i < currentRegion.size(); i++)
      {
         currentRegion.getPoint(i, regionPoint);
         distance.sub(regionPoint, intersectionPoint);

         double orthogonalDistanceFromLine = Math.abs(distance.dot(perpendicularToDirection));
         if (orthogonalDistanceFromLine <= maxDistance)
            points1D.add(distance.dot(intersectionDirection));
      }

      if (points1D.size() < 2)
         return null;

      List<LineSegment1d> intersectionSegments = new ArrayList<>();

      double firstEndpoint = points1D.poll();
      double secondEndpoint = Double.NaN;
      
      double lastPoint1D = firstEndpoint;

      while (!points1D.isEmpty())
      {
         double currentPoint1D = points1D.poll();

         if (Math.abs(currentPoint1D - lastPoint1D) < maxDistance)
         { // The current point is close enough to the previous, we extend the current segment
            secondEndpoint = currentPoint1D;
            if (points1D.isEmpty() && Math.abs(secondEndpoint - firstEndpoint) >= minIntersectionLength)
               intersectionSegments.add(new LineSegment1d(firstEndpoint, secondEndpoint));
         }
         else
         { // The current point is too far from the previous, end of the current segment
            // If there is not secondEndpoint, that means the firstEndpoint is isolated => not an intersection.
            if (!Double.isNaN(secondEndpoint) || Math.abs(secondEndpoint - firstEndpoint) >= minIntersectionLength)
               intersectionSegments.add(new LineSegment1d(firstEndpoint, secondEndpoint));

            if (points1D.size() < 2)
               break;

            // Beginning of a new segment
            firstEndpoint = currentPoint1D;
            secondEndpoint = Double.NaN; // Serve to detect isolated point.
         }
         lastPoint1D = currentPoint1D;
      }

      return intersectionSegments.isEmpty() ? null : intersectionSegments;
   }
}
