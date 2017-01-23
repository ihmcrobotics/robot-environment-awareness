package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.LineSegment3d;

public class PolygonizerTools
{
   public static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toPointsInPlane(pointsToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point2d toPointInPlane(Point3d pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static Point2d toPointInPlane(Point3f pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment2d> toLineSegmentsInPlane(List<LineSegment3d> lineSegmentsToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment2d> toLineSegmentsInPlane(List<LineSegment3d> lineSegmentsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInPlane(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment2d toLineSegmentInPlane(LineSegment3d lineSegmentToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toLineSegmentInPlane(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment2d toLineSegmentInPlane(LineSegment3d lineSegmentToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point2d lineSemgentStart = toPointInPlane(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point2d lineSemgentEnd = toPointInPlane(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment2d(lineSemgentStart, lineSemgentEnd);
   }

   public static Point2d toPointInPlane(double xToTransform, double yToTransform, double zToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point2d pointInPlane = new Point2d();

      double qx = -planeOrientation.getX();
      double qy = -planeOrientation.getY();
      double qz = -planeOrientation.getZ();
      double qs = planeOrientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = xToTransform - planeOrigin.getX();
      double y = yToTransform - planeOrigin.getY();
      double z = zToTransform - planeOrigin.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;

      pointInPlane.setX(x + qs * crossX + crossCrossX);
      pointInPlane.setY(y + qs * crossY + crossCrossY);

      return pointInPlane;
   }

   public static List<Point3d> toPointsInWorld(List<Point2d> pointsInPlane, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toPointsInWorld(pointsInPlane, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static List<Point3d> toPointsInWorld(List<Point2d> pointsInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsInPlane.stream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point3d toPointInWorld(Point2d pointInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInWorld(pointInPlane.getX(), pointInPlane.getY(), planeOrigin, planeOrientation);
   }

   public static List<LineSegment3d> toLineSegmentsInWorld(List<LineSegment2d> lineSegmentsToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeNormal)).collect(Collectors.toList());
   }

   public static List<LineSegment3d> toLineSegmentsInWorld(List<LineSegment2d> lineSegmentsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return lineSegmentsToTransform.stream().map(lineSegment -> toLineSegmentInWorld(lineSegment, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static LineSegment3d toLineSegmentInWorld(LineSegment2d lineSegmentToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toLineSegmentInWorld(lineSegmentToTransform, planeOrigin, getQuaternionFromZUpToVector(planeNormal));
   }

   public static LineSegment3d toLineSegmentInWorld(LineSegment2d lineSegmentToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point3d lineSemgentStart = toPointInWorld(lineSegmentToTransform.getFirstEndpoint(), planeOrigin, planeOrientation);
      Point3d lineSemgentEnd = toPointInWorld(lineSegmentToTransform.getSecondEndpoint(), planeOrigin, planeOrientation);
      return new LineSegment3d(lineSemgentStart, lineSemgentEnd);
   }

   public static Point3d toPointInWorld(double xToTransform, double yToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point3d pointInWorld = new Point3d();

      double qx = planeOrientation.getX();
      double qy = planeOrientation.getY();
      double qz = planeOrientation.getZ();
      double qs = planeOrientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = xToTransform;
      double y = yToTransform;
      double z = 0.0;

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      pointInWorld.setX(x + qs * crossX + crossCrossX);
      pointInWorld.setY(y + qs * crossY + crossCrossY);
      pointInWorld.setZ(z + qs * crossZ + crossCrossZ);
      pointInWorld.add(planeOrigin);

      return pointInWorld;
   }

   public static Quat4d getQuaternionFromZUpToVector(Vector3f normal)
   {
      return getQuaternionFromZUpToVector(new Vector3d(normal));
   }

   public static Quat4d getQuaternionFromZUpToVector(Vector3d normal)
   {
      Quat4d orientation = new Quat4d();
      orientation.set(GeometryTools.getAxisAngleFromZUpToVector(normal));
      return orientation;
   }

   public static double computeEllipsoidVolume(Vector3d radii)
   {
      return computeEllipsoidVolume(radii.getX(), radii.getY(), radii.getZ());
   }

   public static double computeEllipsoidVolume(double xRadius, double yRadius, double zRadius)
   {
      return 4.0 / 3.0 * Math.PI * xRadius * yRadius * zRadius;
   }
}
