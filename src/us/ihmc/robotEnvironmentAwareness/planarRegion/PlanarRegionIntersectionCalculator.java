package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.Line3d;
import us.ihmc.robotics.geometry.LineSegment1d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.LineSegment3d;

public class PlanarRegionIntersectionCalculator
{
   public static List<LineSegment3d> computeIntersections(List<PlanarRegionSegmentationRawData> rawData, IntersectionEstimationParameters parameters)
   {
      List<LineSegment3d> result = new ArrayList<>();

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

            double minRegionAngleDifference = parameters.getMinRegionAngleDifference();

            Line3d intersectionLine = computeIntersectionLine3d(currentRegion, currentNeighbor, minRegionAngleDifference);
            if (intersectionLine == null)
               continue;

            double maxDistanceToRegion = parameters.getMaxDistanceToRegion();
            double minIntersectionLength = parameters.getMinIntersectionLength();

            List<LineSegment3d> intersectionList = findIntersectionEndPoints(currentRegion, currentNeighbor, maxDistanceToRegion, minIntersectionLength,
                                                                             intersectionLine);

            if (intersectionList != null)
            {
               result.addAll(intersectionList);

               if (parameters.isAddIntersectionsToRegions())
               {
                  List<LineSegment2d> intersectionsForCurrentRegion = PolygonizerTools.toLineSegmentsInPlane(intersectionList, currentRegion.getOrigin(), currentRegion.getNormal());
                  currentRegion.addIntersections(intersectionsForCurrentRegion);
                  List<LineSegment2d> intersectionsForCurrentNeighbor = PolygonizerTools.toLineSegmentsInPlane(intersectionList, currentNeighbor.getOrigin(), currentNeighbor.getNormal());
                  currentNeighbor.addIntersections(intersectionsForCurrentNeighbor);
               }
            }
         }
      }

      return result;
   }

   private static Line3d computeIntersectionLine3d(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor,
                                                   double minRegionAngleDifference)
   {
      Point3d origin1 = currentRegion.getOrigin();
      Vector3d normal1 = currentRegion.getNormal();
      Point3d origin2 = currentNeighbor.getOrigin();
      Vector3d normal2 = currentNeighbor.getNormal();

      Point3d pointOnIntersection = new Point3d();
      Vector3d intersectionDirection = new Vector3d();
      boolean success = GeometryTools.getIntersectionBetweenTwoPlanes(origin1, normal1, origin2, normal2, minRegionAngleDifference, pointOnIntersection,
                                                                      intersectionDirection);
      if (!success)
         return null;
      else
         return new Line3d(pointOnIntersection, intersectionDirection);
   }

   private static List<LineSegment3d> findIntersectionEndPoints(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor,
                                                                double maxDistance, double minIntersectionLength, Line3d intersectionLine)
   {

      List<LineSegment1d> intersectionsFromRegion1 = findIntersectionLineSegments(currentRegion, maxDistance, minIntersectionLength, intersectionLine);
      if (intersectionsFromRegion1 == null || intersectionsFromRegion1.isEmpty())
         return null;
      List<LineSegment1d> intersectionsFromRegion2 = findIntersectionLineSegments(currentNeighbor, maxDistance, minIntersectionLength, intersectionLine);
      if (intersectionsFromRegion2 == null || intersectionsFromRegion2.isEmpty())
         return null;

      List<LineSegment3d> intersections = new ArrayList<>();

      for (LineSegment1d intersectionFromRegion1 : intersectionsFromRegion1)
      {
         for (LineSegment1d intersectionFromRegion2 : intersectionsFromRegion2)
         {
            LineSegment1d overlap = intersectionFromRegion1.computeOverlap(intersectionFromRegion2);
            if (overlap != null && overlap.length() > minIntersectionLength)
               intersections.add(overlap.toLineSegment3d(intersectionLine));
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
   private static List<LineSegment1d> findIntersectionLineSegments(PlanarRegionSegmentationRawData currentRegion, double maxDistance,
                                                                   double minIntersectionLength, Line3d intersectionLine)
   {
      Vector3d perpendicularToDirection = new Vector3d();
      perpendicularToDirection.cross(currentRegion.getNormal(), intersectionLine.getNormalizedVector());
      perpendicularToDirection.normalize();

      Vector3d distance = new Vector3d();
      Point3d regionPoint = new Point3d();

      // 1-D Coordinates along the intersection direction of all the region points that are close enough to the intersection.
      // By using a PriorityQueue the coordinates are sorted.
      PriorityQueue<Double> points1D = new PriorityQueue<>();

      for (int i = 0; i < currentRegion.size(); i++)
      {
         currentRegion.getPoint(i, regionPoint);
         distance.sub(regionPoint, intersectionLine.getPoint());

         double orthogonalDistanceFromLine = Math.abs(distance.dot(perpendicularToDirection));
         if (orthogonalDistanceFromLine <= maxDistance)
            points1D.add(distance.dot(intersectionLine.getNormalizedVector()));
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
