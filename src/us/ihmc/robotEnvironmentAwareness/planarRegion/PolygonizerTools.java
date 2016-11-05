package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotics.geometry.GeometryTools;

public class PolygonizerTools
{
   public static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsToTransform.stream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static List<Point2d> extractPointsInPlane(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      Point3d planeOrigin = ocTreeNodePlanarRegion.getOrigin();
      Quat4d planeOrientation = new Quat4d();
      planeOrientation.set(GeometryTools.getRotationBasedOnNormal(ocTreeNodePlanarRegion.getNormal()));
      return ocTreeNodePlanarRegion.nodeStream().map(node -> toPointInPlane(node, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point2d toPointInPlane(Point3d pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return toPointInPlane(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ(), planeOrigin, planeOrientation);
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

   public static List<Point3d> toPointsInWorld(List<Point2d> pointsInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      return pointsInPlane.stream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).collect(Collectors.toList());
   }

   public static Point3d toPointInWorld(Point2d pointInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point3d pointInWorld = new Point3d();

      double qx = planeOrientation.getX();
      double qy = planeOrientation.getY();
      double qz = planeOrientation.getZ();
      double qs = planeOrientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = pointInPlane.getX();
      double y = pointInPlane.getY();
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
}
