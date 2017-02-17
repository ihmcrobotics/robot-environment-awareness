package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;

public class PlanarRegionSegmentationRawData
{
   private final int regionId;
   private final Vector3D normal;
   private final Point3D origin;
   private final List<Point3D> pointCloud;
   private final Quaternion orientation;

   public PlanarRegionSegmentationRawData(int regionId, Vector3D32 normal, Point3D32 origin)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      pointCloud = new ArrayList<>();
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D normal, Point3D origin)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      pointCloud = new ArrayList<>();
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D32 normal, Point3D32 origin, List<Point3D32> pointCloud)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      this.pointCloud = pointCloud.stream().map(point3f -> new Point3D(point3f)).collect(Collectors.toList());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3D normal, Point3D origin, List<Point3D> pointCloud)
   {
      this.regionId = regionId;
      this.normal = new Vector3D(normal);
      this.origin = new Point3D(origin);
      this.pointCloud = pointCloud.stream().map(point -> new Point3D(point)).collect(Collectors.toList());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationNodeData ocTreeNodePlanarRegion)
   {
      regionId = ocTreeNodePlanarRegion.getId();
      normal = new Vector3D(ocTreeNodePlanarRegion.getNormal());
      origin = new Point3D(ocTreeNodePlanarRegion.getOrigin());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
      pointCloud = ocTreeNodePlanarRegion.nodeStream()
                                         .map(NormalOcTreeNode::getHitLocationCopy)
                                         .collect(Collectors.toList());
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationMessage message)
   {
      regionId = message.getRegionId();
      normal = new Vector3D(message.getNormal());
      origin = new Point3D(message.getOrigin());
      orientation = PolygonizerTools.getRotationBasedOnNormal(normal);
      pointCloud = Arrays.stream(message.getHitLocations())
                         .map(point3f -> new Point3D(point3f))
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

   public List<Point2D> getPointCloudInPlane()
   {
      return pointCloud.stream()
                       .map(this::toPointInPlane)
                       .collect(Collectors.toList());
   }

   private Point2D toPointInPlane(Point3D point3d)
   {
      return PolygonizerTools.toPointInPlane(point3d, origin, orientation);
   }

   public List<Point3D> getPointCloudInWorld()
   {
      return pointCloud;
   }

   public void getPoint(int index, Point3D pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public Point3D getOrigin()
   {
      return origin;
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public Stream<Point3D> stream()
   {
      return pointCloud.stream();
   }

   public Stream<Point3D> parallelStream()
   {
      return pointCloud.parallelStream();
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
