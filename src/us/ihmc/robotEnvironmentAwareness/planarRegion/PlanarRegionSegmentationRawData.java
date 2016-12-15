package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionSegmentationRawData
{
   private final int regionId;
   private final Vector3d normal;
   private final Point3d origin;
   private final List<Point3d> pointCloud;
   private final Quat4d orientation;

   public PlanarRegionSegmentationRawData(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      regionId = ocTreeNodePlanarRegion.getId();
      normal = new Vector3d(ocTreeNodePlanarRegion.getNormal());
      origin = new Point3d(ocTreeNodePlanarRegion.getOrigin());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
      pointCloud = ocTreeNodePlanarRegion.nodeStream()
                                         .map(NormalOcTreeNode::getHitLocationCopy)
                                         .collect(Collectors.toList());
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationMessage message)
   {
      regionId = message.getRegionId();
      normal = new Vector3d(message.getNormal());
      origin = new Point3d(message.getOrigin());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
      pointCloud = Arrays.stream(message.getHitLocations())
                         .map(point3f -> new Point3d(point3f))
                         .collect(Collectors.toList());
   }

   public int getRegionId()
   {
      return regionId;
   }

   public int size()
   {
      return pointCloud.size();
   }

   public List<Point2d> getPointCloudInPlane()
   {
      return pointCloud.stream()
                       .map(this::toPointInPlane)
                       .collect(Collectors.toList());
   }

   private Point2d toPointInPlane(Point3d point3d)
   {
      return PolygonizerTools.toPointInPlane(point3d, origin, orientation);
   }

   public List<Point3d> getPointCloudInWorld()
   {
      return pointCloud;
   }

   public void getPoint(int index, Point3d pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public Point3d getOrigin()
   {
      return origin;
   }

   public Vector3d getNormal()
   {
      return normal;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public RigidBodyTransform getTransformFromLocalToWorld()
   {
      return new RigidBodyTransform(orientation, origin);
   }

   public PlanarRegionSegmentationMessage toMessage()
   {
      return new PlanarRegionSegmentationMessage(regionId, origin, normal, null, pointCloud);
   }

   public static PlanarRegionSegmentationMessage[] toMessageArray(List<PlanarRegionSegmentationRawData> rawData)
   {
      return rawData.stream()
                    .map(PlanarRegionSegmentationRawData::toMessage)
                    .toArray(size -> new PlanarRegionSegmentationMessage[size]);
   }
}
