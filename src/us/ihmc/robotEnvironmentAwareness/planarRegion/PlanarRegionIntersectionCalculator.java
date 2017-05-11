package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.LineSegment1D;

public class PlanarRegionIntersectionCalculator
{
   public static List<LineSegment3D> computeIntersections(List<PlanarRegionSegmentationRawData> rawData, IntersectionEstimationParameters parameters)
   {
      List<LineSegment3D> result = new ArrayList<>();

      Point3D intersectionPoint = new Point3D();
      Vector3D intersectionDirection = new Vector3D();

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

            List<LineSegment3D> intersectionList = findIntersectionEndPoints(currentRegion, currentNeighbor, maxDistanceToRegion, minIntersectionLength, intersectionPoint, intersectionDirection);

            if (intersectionList != null)
            {
               Vector3D intersectionLength = new Vector3D();

               result.addAll(intersectionList);

               if (parameters.isAddIntersectionsToRegions())
               {
                  for (LineSegment3D intersection : intersectionList)
                  {
                     intersectionLength.sub(intersection.getSecondEndpoint(), intersection.getFirstEndpoint());
                     double length = intersectionLength.length();
                     double delta = 0.01;
                     int numberOfPointsToAdd = (int) (length / delta);
                     for (int k = 0; k < numberOfPointsToAdd; k++)
                     {
                        Point3D newPoint = new Point3D();
                        double alpha = k / (double) numberOfPointsToAdd;
                        newPoint.scaleAdd(alpha, intersectionLength, intersection.getFirstEndpoint());
                        // FIXME Figure out something less hackish
//                        currentRegion.addPoint(new Point3D(newPoint));
//                        currentNeighbor.addPoint(new Point3D(newPoint));
                     }
                  }
               }
            }
         }
      }

      return result;
   }

   private static boolean computeIntersectionPointAndDirection(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor, Point3D intersectionPointToPack,
         Vector3D intersectionDirectionToPack, double minRegionAngleDifference)
   {
      Point3D origin1 = currentRegion.getOrigin();
      Vector3D normal1 = currentRegion.getNormal();
      Point3D origin2 = currentNeighbor.getOrigin();
      Vector3D normal2 = currentNeighbor.getNormal();

      double angle = normal1.angle(normal2);

      if (MathTools.epsilonEquals(angle, 0.0, minRegionAngleDifference))// || MathTools.epsilonEquals(Math.abs(angle), Math.PI, epsilon))
         return false;

      intersectionDirectionToPack.cross(normal1, normal2);
      double det = intersectionDirectionToPack.lengthSquared();

      double d1 = normal1.dot(new Vector3D(origin1));
      double d2 = normal2.dot(new Vector3D(origin2));

      Vector3D normal3Cross2 = new Vector3D();
      normal3Cross2.cross(intersectionDirectionToPack, normal2);
      Vector3D normal1Cross3 = new Vector3D();
      normal1Cross3.cross(normal1, intersectionDirectionToPack);
      Vector3D normal2Cross1 = new Vector3D();
      normal2Cross1.cross(normal2, normal1);

      intersectionPointToPack.setAndScale(d1, normal3Cross2);
      intersectionPointToPack.scaleAdd(d2, normal1Cross3, intersectionPointToPack);
      intersectionDirectionToPack.normalize();

      double d3 = 0.5 * (intersectionDirectionToPack.dot(new Vector3D(origin1)) + intersectionDirectionToPack.dot(new Vector3D(origin2)));
      intersectionPointToPack.scaleAdd(d3, normal2Cross1, intersectionPointToPack);

      intersectionPointToPack.scale(-1.0 / det);
      return true;
   }

   private static List<LineSegment3D> findIntersectionEndPoints(PlanarRegionSegmentationRawData currentRegion, PlanarRegionSegmentationRawData currentNeighbor, double maxDistance, double minIntersectionLength, Point3D intersectionPoint,
         Vector3D intersectionDirection)
   {

      List<LineSegment1D> intersectionsFromRegion1 = findIntersectionLineSegments(currentRegion, maxDistance, minIntersectionLength, intersectionPoint, intersectionDirection);
      if (intersectionsFromRegion1 == null || intersectionsFromRegion1.isEmpty())
         return null;
      List<LineSegment1D> intersectionsFromRegion2 = findIntersectionLineSegments(currentNeighbor, maxDistance, minIntersectionLength, intersectionPoint, intersectionDirection);
      if (intersectionsFromRegion2 == null || intersectionsFromRegion2.isEmpty())
         return null;

      List<LineSegment3D> intersections = new ArrayList<>();

      for (LineSegment1D intersectionFromRegion1 : intersectionsFromRegion1)
      {
         for (LineSegment1D intersectionFromRegion2 : intersectionsFromRegion2)
         {
            LineSegment1D overlap = intersectionFromRegion1.computeOverlap(intersectionFromRegion2);
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
   private static List<LineSegment1D> findIntersectionLineSegments(PlanarRegionSegmentationRawData currentRegion, double maxDistance, double minIntersectionLength, Point3D intersectionPoint, Vector3D intersectionDirection)
   {
      Vector3D perpendicularToDirection = new Vector3D();
      perpendicularToDirection.cross(currentRegion.getNormal(), intersectionDirection);
      perpendicularToDirection.normalize();

      Vector3D distance = new Vector3D();
      Point3D regionPoint = new Point3D();

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

      List<LineSegment1D> intersectionSegments = new ArrayList<>();

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
               intersectionSegments.add(new LineSegment1D(firstEndpoint, secondEndpoint));
         }
         else
         { // The current point is too far from the previous, end of the current segment
            // If there is not secondEndpoint, that means the firstEndpoint is isolated => not an intersection.
            if (!Double.isNaN(secondEndpoint) || Math.abs(secondEndpoint - firstEndpoint) >= minIntersectionLength)
               intersectionSegments.add(new LineSegment1D(firstEndpoint, secondEndpoint));

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
