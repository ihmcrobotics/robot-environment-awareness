package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.LineSegment3d;

public class PlanarRegionIntersectionCalculator
{
   private final List<LineSegment3d> intersections = new ArrayList<>();
   private final IntersectionEstimationParameters parameters = new IntersectionEstimationParameters();

   public void compute(List<PlanarRegion> planarRegions)
   {
      Point3d intersectionPoint = new Point3d();
      Vector3d intersectionDirection = new Vector3d();
      clear();

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion currentRegion = planarRegions.get(i);

         if (currentRegion.getNumberOfNodes() < parameters.getMinRegionSize())
            continue;

         for (int j = i + 1; j < planarRegions.size(); j++)
         {
            PlanarRegion currentNeighbor = planarRegions.get(j);

            if (currentNeighbor.getNumberOfNodes() < parameters.getMinRegionSize())
               continue;

            boolean success = computeIntersectionPointAndDirection(currentRegion, currentNeighbor, intersectionPoint, intersectionDirection);
            if (!success)
               continue;

            LineSegment3d intersection = findIntersectionEndPoints(currentRegion, currentNeighbor, parameters.getMaxDistanceToRegion(), intersectionPoint,
                  intersectionDirection);
            if (intersection != null)
            {
               Vector3d intersectionLength = new Vector3d();
               intersectionLength.sub(intersection.getPointB(), intersection.getPointA());
               double length = intersectionLength.length();

               if (length < parameters.getMinIntersectionLength())
                  continue;

               intersections.add(intersection);

               if (parameters.isAddIntersectionsToRegions())
               {
                  double delta = 0.04;
                  int numberOfPointsToAdd = (int) (length / delta);
                  numberOfPointsToAdd = Math.min(10, numberOfPointsToAdd);
                  for (int k = 0; k < numberOfPointsToAdd; k++)
                  {
                     Point3d newPoint = new Point3d();
                     double alpha = k / (double) numberOfPointsToAdd;
                     newPoint.scaleAdd(alpha, intersectionLength, intersection.getPointA());
                     currentRegion.addPoint(new Point3d(newPoint));
                     currentNeighbor.addPoint(new Point3d(newPoint));
                  }
               }
            }
         }
      }
      System.out.println("Number of intersections: " + intersections.size());
   }

   public void clear()
   {
      intersections.clear();
   }

   private boolean computeIntersectionPointAndDirection(PlanarRegion region1, PlanarRegion region2, Point3d intersectionPointToPack,
         Vector3d intersectionDirectionToPack)
   {
      Point3d origin1 = region1.getOrigin();
      Vector3d normal1 = region1.getNormal();
      Point3d origin2 = region2.getOrigin();
      Vector3d normal2 = region2.getNormal();

      double angle = normal1.angle(normal2);

      if (MathTools.epsilonEquals(angle, 0.0, parameters.getMinRegionAngleDifference()))// || MathTools.epsilonEquals(Math.abs(angle), Math.PI, epsilon))
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

   private LineSegment3d findIntersectionEndPoints(PlanarRegion region1, PlanarRegion region2, double maxDistance, Point3d intersectionPoint,
         Vector3d intersectionDirection)
   {
      LineSegment3d intersection1 = findIntersectionEndPoints(region1, maxDistance, intersectionPoint, intersectionDirection);
      if (intersection1 == null)
         return null;
      LineSegment3d intersection2 = findIntersectionEndPoints(region2, maxDistance, intersectionPoint, intersectionDirection);
      if (intersection2 == null)
         return null;

      LineSegment3d intersection = new LineSegment3d();

      Vector3d direction = new Vector3d();

      Point3d pointA1 = intersection1.getPointA();
      Point3d pointB1 = intersection1.getPointB();
      Point3d pointA2 = intersection2.getPointA();
      Point3d pointB2 = intersection2.getPointB();

      // First verify that the two segments overlap
      direction.sub(pointA2, pointA1);
      double distanceA1ToA2 = direction.dot(intersectionDirection);
      direction.sub(pointB2, pointA1);
      double distanceA1ToB2 = direction.dot(intersectionDirection);
      direction.sub(pointA2, pointB1);
      double distanceB1ToA2 = direction.dot(intersectionDirection);
      direction.sub(pointB2, pointB1);
      double distanceB1ToB2 = direction.dot(intersectionDirection);

      boolean isA1BetweenA2AndB2 = distanceA1ToA2 * distanceA1ToB2 < 0.0;
      boolean isB1BetweenA2AndB2 = distanceB1ToA2 * distanceB1ToB2 < 0.0;

      boolean isA2BetweenA1AndB1 = distanceA1ToA2 * distanceB1ToA2 < 0.0;
      boolean isB2BetweenA1AndB1 = distanceA1ToB2 * distanceB1ToB2 < 0.0;

      if (!isA1BetweenA2AndB2 && !isB1BetweenA2AndB2 && !isA2BetweenA1AndB1 && !isB2BetweenA1AndB1)
         return null; // They do not overlap

      direction.sub(pointB1, pointA1);
      double distanceA1ToB1 = direction.dot(intersectionDirection);

      direction.sub(pointB2, pointA2);
      double distanceA2ToB2 = direction.dot(intersectionDirection);

      if (distanceA1ToB1 * distanceA2ToB2 > 0.0)
      { // The two segments point to the same direction
         if (isA1BetweenA2AndB2)
            intersection.setPointA(pointA1);
         else
            intersection.setPointA(pointA2);

         if (isB1BetweenA2AndB2)
            intersection.setPointB(pointB1);
         else
            intersection.setPointB(pointB2);
      }
      else
      { // The two segments point to opposite directions
         if (isA1BetweenA2AndB2)
            intersection.setPointA(pointA1);
         else
            intersection.setPointA(pointB2);

         if (isB1BetweenA2AndB2)
            intersection.setPointB(pointB1);
         else
            intersection.setPointB(pointA2);
      }

      return intersection;
   }

   private LineSegment3d findIntersectionEndPoints(PlanarRegion region, double maxDistance, Point3d intersectionPoint, Vector3d intersectionDirection)
   {

      Vector3d directionA = new Vector3d(intersectionDirection);
      directionA.normalize();
      Vector3d directionB = new Vector3d();
      directionB.negate(directionA);

      double distanceToA = Double.NEGATIVE_INFINITY;
      double distanceToB = Double.NEGATIVE_INFINITY;

      Vector3d perpendicularToDirection = new Vector3d();
      perpendicularToDirection.cross(region.getNormal(), directionA);
      perpendicularToDirection.normalize();

      Vector3d distance = new Vector3d();

      for (int i = 0; i < region.getNumberOfNodes(); i++)
      {
         Point3d regionPoint = region.getPoint(i);

         distance.sub(regionPoint, intersectionPoint);

         if (Math.abs(distance.dot(perpendicularToDirection)) > maxDistance)
            continue;

         if (distance.dot(directionA) > distanceToA)
            distanceToA = distance.dot(directionA);
         else if (distance.dot(directionB) > distanceToB)
            distanceToB = distance.dot(directionB);
      }

      if (Double.isInfinite(distanceToA) || Double.isInfinite(distanceToB))
         return null;

      Point3d pointA = new Point3d();
      Point3d pointB = new Point3d();
      pointA.scaleAdd(distanceToA, directionA, intersectionPoint);
      pointB.scaleAdd(distanceToB, directionB, intersectionPoint);

      return new LineSegment3d(pointA, pointB);
   }

   private double findMinDistanceToPoint(PlanarRegion planarRegion, Point3d point)
   {
      if (planarRegion.getNumberOfNodes() == 0)
         return Double.POSITIVE_INFINITY;

      double minDistanceSquared = Double.POSITIVE_INFINITY;

      for (int i = 0; i < planarRegion.getNumberOfNodes(); i++)
      {
         Point3d candidate = planarRegion.getPoint(i);

         double distance = candidate.distanceSquared(point);
         if (distance < minDistanceSquared)
            minDistanceSquared = distance;
      }

      return minDistanceSquared;
   }

   private Point3d findFarthestPointInDirection(PlanarRegion planarRegion, Vector3d direction)
   {
      if (planarRegion.getNumberOfNodes() == 0)
         return null;

      Point3d result = new Point3d();

      double maxDistance = Double.NEGATIVE_INFINITY;
      direction.normalize();
      Vector3d translation = new Vector3d();
      Point3d origin = planarRegion.getOrigin();

      for (int i = 0; i < planarRegion.getNumberOfNodes(); i++)
      {
         Point3d candidate = planarRegion.getPoint(i);
         translation.sub(candidate, origin);
         double distance = translation.dot(direction);
         if (distance > maxDistance)
         {
            maxDistance = distance;
            result.set(candidate);
         }
      }

      return result;
   }

   public void setParameters(IntersectionEstimationParameters parameters)
   {
      this.parameters.set(parameters);
   }

   public int getNumberOfIntersections()
   {
      return intersections.size();
   }

   public LineSegment3d getIntersection(int index)
   {
      return intersections.get(index);
   }
}
