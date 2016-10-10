package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.geometry.PointMean;
import us.ihmc.robotEnvironmentAwareness.geometry.VectorMean;

public class PlanarRegion implements Iterable<NormalOcTreeNode>
{
   public static final int NO_REGION_ID = Integer.MIN_VALUE;

   private int id = NO_REGION_ID;

   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3d temporaryVector = new Vector3d();

   private final List<NormalOcTreeNode> nodes = new ArrayList<>();
   private final Set<NormalOcTreeNode> nodeSet = new HashSet<>();

   public PlanarRegion(int id)
   {
      this.id = id;
   }

   public PlanarRegion(int id, Collection<NormalOcTreeNode> nodes)
   {
      this(id);
      addNodes(nodes);
   }

   public boolean addNode(NormalOcTreeNode node)
   {
      boolean isRegionModified = nodeSet.add(node);
      if (isRegionModified)
      {
         updateNormalAndOriginOnly(node);
         nodes.add(node);
      }
      return isRegionModified;
   }

   public boolean addNodes(Collection<NormalOcTreeNode> nodes)
   {
      return nodes.stream().filter(this::addNode).findFirst().isPresent();
   }

   public boolean addNodesFromOtherRegion(PlanarRegion other)
   {
      return other.nodeStream().filter(this::addNode).findFirst().isPresent();
   }

   public boolean contains(NormalOcTreeNode node)
   {
      return nodeSet.contains(node);
   }

   public void recomputeNormalAndOrigin()
   {
      point.clear();
      normal.clear();
      nodeStream().forEach(this::updateNormalAndOriginOnly);
   }

   private void updateNormalAndOriginOnly(NormalOcTreeNode node)
   {
      node.getNormal(temporaryVector);
      // TODO Review and possibly improve dealing with normal flips.
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         temporaryVector.negate();
      normal.update(temporaryVector);
      point.update(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
   }

   public double distanceFromCenterSquared(Point3d point)
   {
      return this.point.distanceSquared(point);
   }

   public double orthogonalDistance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(Point3d point)
   {
      return Math.abs(orthogonalDistance(point));
   }

   public double orhtogonalDistance(NormalOcTreeNode node)
   {
      temporaryVector.set(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
      temporaryVector.sub(point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(NormalOcTreeNode node)
   {
      return Math.abs(orhtogonalDistance(node));
   }

   public double angle(Vector3d normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3d normal)
   {
      return Math.abs(angle(normal));
   }

   public double dot(Vector3d normal)
   {
      return this.normal.dot(normal);
   }

   public double absoluteDot(Vector3d normal)
   {
      return Math.abs(dot(normal));
   }

   public double dot(PlanarRegion other)
   {
      return dot(other.normal);
   }

   public double absoluteDot(PlanarRegion other)
   {
      return Math.abs(dot(other));
   }

   public double dotWithNodeNormal(NormalOcTreeNode node)
   {
      return normal.getX() * node.getNormalX() + normal.getY() * node.getNormalY() + normal.getZ() * node.getNormalZ();
   }

   public double absoluteDotWithNodeNormal(NormalOcTreeNode node)
   {
      return Math.abs(dotWithNodeNormal(node));
   }

   public Vector3d getNormal()
   {
      return normal;
   }

   public Point3d getOrigin()
   {
      return point;
   }

   public void getPoint(int index, Point3d pointToPack)
   {
      nodes.get(index).getHitLocation(pointToPack);
   }

   public int getId()
   {
      return id;
   }

   public NormalOcTreeNode getNode(int index)
   {
      return nodes.get(index);
   }

   public void clearRegion()
   {
      nodes.clear();
      point.clear();
      normal.clear();
   }

   public boolean containsNode(NormalOcTreeNode node)
   {
      return nodeSet.contains(node);
   }

   public void removeNode(int index)
   {
      NormalOcTreeNode removedNode = nodes.remove(index);
      nodeSet.remove(removedNode);
   }

   public void removeNodesAndUpdate(Collection<NormalOcTreeNode> nodesToRemove)
   {
      boolean containsAtLeastOne = nodesToRemove.parallelStream()
                                                .filter(nodeSet::contains)
                                                .findFirst()
                                                .isPresent();

      if (containsAtLeastOne)
      {
         nodes.removeAll(nodesToRemove);
         recomputeNormalAndOrigin();
      }
   }

   public int getNumberOfNodes()
   {
      return nodes.size();
   }

   public Stream<NormalOcTreeNode> nodeStream()
   {
      return nodes.stream();
   }

   public Stream<NormalOcTreeNode> nodeParallelStream()
   {
      return nodes.parallelStream();
   }

   @Override
   public Iterator<NormalOcTreeNode> iterator()
   {
      return nodes.iterator();
   }

   public void printPointsToFile()
   {
      try
      {
         FileWriter fw = new FileWriter("regionPoints");
         for (NormalOcTreeNode node : this)
            fw.write(node.getHitLocationX() + ", " + node.getHitLocationY() + ", " + node.getHitLocationZ() + "\n");
         fw.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public String toString()
   {
      String ret = "Region ID: " + id;
      ret += ", origin: " + point + ", normal: " + normal;
      ret += ", size: " + getNumberOfNodes();
      return ret;
   }
}
