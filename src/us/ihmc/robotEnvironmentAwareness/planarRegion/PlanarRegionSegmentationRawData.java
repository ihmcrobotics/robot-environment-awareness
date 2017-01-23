package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionSegmentationRawData
{
   private final int regionId;
   private final Vector3d normal;
   private final Point3d origin;
   private final List<Point3d> pointCloud;
   private final Quat4d orientation;
   private final List<LineSegment2d> intersections;

   public PlanarRegionSegmentationRawData(int regionId, Vector3f normal, Point3f origin)
   {
      this(regionId, normal, origin, Collections.emptyList());
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3d normal, Point3d origin)
   {
      this(regionId, normal, origin, Collections.emptyList());
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationNodeData nodeData)
   {
      this(nodeData.getId(), nodeData.getNormal(), nodeData.getOrigin(), nodeData.nodeStream(), null);
   }

   public PlanarRegionSegmentationRawData(PlanarRegionSegmentationMessage message)
   {
      this(message.getRegionId(), message.getNormal(), message.getOrigin(), Arrays.stream(message.getHitLocations()), null);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3f normal, Point3f origin, List<Point3f> pointCloud)
   {
      this(regionId, normal, origin, pointCloud.stream(), null);
   }

   public PlanarRegionSegmentationRawData(int regionId, Vector3d normal, Point3d origin, List<Point3d> pointCloud)
   {
      this(regionId, normal, origin, pointCloud.stream(), null);
   }

   private <T> PlanarRegionSegmentationRawData(int regionId, Vector3f normal, Point3f origin, Stream<T> streamToConvert, List<LineSegment2d> intersections)
   {
      this(regionId, new Vector3d(normal), new Point3d(origin), streamToConvert, intersections);
   }

   private <T> PlanarRegionSegmentationRawData(int regionId, Vector3d normal, Point3d origin, Stream<T> streamToConvert, List<LineSegment2d> intersections)
   {
      this.regionId = regionId;
      this.normal = new Vector3d(normal);
      this.origin = new Point3d(origin);
      this.pointCloud = toListOfPoint3d(streamToConvert);
      orientation = PolygonizerTools.getQuaternionFromZUpToVector(normal);
      if (intersections == null)
         this.intersections = new ArrayList<>();
      else
         this.intersections = intersections.stream().map(LineSegment2d::new).collect(Collectors.toList());
   }

   private static <T> List<Point3d> toListOfPoint3d(Stream<T> inputStream)
   {
      if (inputStream == null)
         return Collections.emptyList();
      else
         return inputStream.map(PlanarRegionSegmentationRawData::convertToPoint3d).collect(Collectors.toList());
   }

   private static <T> Point3d convertToPoint3d(T input)
   {
      if (input instanceof Point3d)
         return new Point3d((Point3d) input);
      
      if (input instanceof Point3f)
         return new Point3d((Point3f) input);

      if (input instanceof NormalOcTreeNode)
         return new Point3d(((NormalOcTreeNode) input).getHitLocationCopy());

      throw new RuntimeException("Unhandled type: " + input.getClass().getSimpleName());
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

   public Stream<Point3d> stream()
   {
      return pointCloud.stream();
   }

   public Stream<Point3d> parallelStream()
   {
      return pointCloud.parallelStream();
   }

   public RigidBodyTransform getTransformFromLocalToWorld()
   {
      return new RigidBodyTransform(orientation, origin);
   }

   public boolean hasIntersections()
   {
      return intersections != null;
   }

   public void addIntersections(List<LineSegment2d> intersectionsToAdd)
   {
      intersectionsToAdd.forEach(this::addIntersection);
   }

   public void addIntersection(LineSegment2d intersectionToAdd)
   {
      intersections.add(intersectionToAdd);
   }

   public List<LineSegment2d> getIntersections()
   {
      return intersections;
   }

   public PlanarRegionSegmentationMessage toMessage()
   {
      return new PlanarRegionSegmentationMessage(regionId, origin, normal, null, pointCloud);
   }

   public static PlanarRegionSegmentationMessage[] toMessageArray(List<PlanarRegionSegmentationRawData> rawData)
   {
      return rawData.stream()
                    .map(PlanarRegionSegmentationRawData::toMessage)
                    .toArray(PlanarRegionSegmentationMessage[]::new);
   }
}
