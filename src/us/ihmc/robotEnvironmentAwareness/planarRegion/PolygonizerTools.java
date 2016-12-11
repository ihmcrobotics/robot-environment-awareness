package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;

public class PolygonizerTools
{
   public static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toPointsInPlane(pointsToTransform, planeOrigin, getRotationBasedOnNormal(planeNormal));
   }

   public static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static List<Point2d> extractPointsInPlane(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      Point3d planeOrigin = ocTreeNodePlanarRegion.getOrigin();
      Quat4d planeOrientation = getRotationBasedOnNormal(ocTreeNodePlanarRegion.getNormal());
      return ocTreeNodePlanarRegion.nodeStream().map(node -> toPointInPlane(node, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static List<Point2d> extractPointsInPlane(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      return Arrays.stream(planarRegionSegmentationMessage.getHitLocations()).map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point2d toPointInPlane(Point3d pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static Point2d toPointInPlane(Point3f pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
   }

   public static Point2d toPointInPlane(NormalOcTreeNode nodeToTransform, Point3d planeOrigin, Vector3d planeNormal)
   {
      return toPointInPlane(nodeToTransform, planeOrigin, getRotationBasedOnNormal(planeNormal));
   }

   public static Point2d toPointInPlane(NormalOcTreeNode nodeToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(nodeToTransform.getHitLocationX(), nodeToTransform.getHitLocationY(), nodeToTransform.getHitLocationZ(), planeOrigin,
            planeOrientation);
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
      return toPointsInWorld(pointsInPlane, planeOrigin, getRotationBasedOnNormal(planeNormal));
   }

   public static List<Point3d> toPointsInWorld(List<Point2d> pointsInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsInPlane.stream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point3d toPointInWorld(Point2d pointInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInWorld(pointInPlane.getX(), pointInPlane.getY(), planeOrigin, planeOrientation);
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

   public static Quat4d getRotationBasedOnNormal(Vector3f normal)
   {
      return getRotationBasedOnNormal(new Vector3d(normal));
   }

   public static Quat4d getRotationBasedOnNormal(Vector3d normal)
   {
      Quat4d orientation = new Quat4d();
      orientation.set(JOctoMapGeometryTools.getRotationBasedOnNormal(normal));
      return orientation;
   }
}
