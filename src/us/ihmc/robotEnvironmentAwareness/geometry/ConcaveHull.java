package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ConcaveHull implements Iterable<Point2d>
{
   private final List<Point2d> hullVertices;

   public ConcaveHull(List<Point2d> hullVertices)
   {
      this.hullVertices = hullVertices;
   }

   public ConcaveHull(ConcaveHull other)
   {
      this.hullVertices = new ArrayList<>();
      other.forEach(hullVertices::add);
   }

   public void ensureCounterClockwiseOrdering()
   {
      ConcaveHullTools.ensureCounterClockwiseOrdering(hullVertices);
   }

   public void ensureClockwiseOrdering()
   {
      ConcaveHullTools.ensureClockwiseOrdering(hullVertices);
   }

   public void removeSuccessiveDuplicateVertices()
   {
      ConcaveHullTools.removeSuccessiveDuplicateVertices(hullVertices);
   }

   /**
    * Returns true only if removing the vertex would generate a kink in the concave polygon.
    * Meaning, it would cause several edges to cross each other.
    */
   public boolean isVertexPreventingKink(int vertexIndex)
   {
      return ConcaveHullTools.isVertexPreventingKink(vertexIndex, hullVertices);
   }

   public double computePerimeter()
   {
      return ConcaveHullTools.computePerimeter(hullVertices);
   }

   public boolean computeConcaveHullPocket(int concaveVertexIndex, ConcaveHullPocket pocketToPack)
   {
      return ConcaveHullTools.computeConcaveHullPocket(concaveVertexIndex, pocketToPack, hullVertices);
   }

   public ConcaveHullPocket computeConcaveHullPocket(int concaveVertexIndex)
   {
      return ConcaveHullTools.computeConcaveHullPocket(concaveVertexIndex, hullVertices);
   }

   public Set<ConcaveHullPocket> findConcaveHullPockets(double depthThreshold)
   {
      return ConcaveHullTools.findConcaveHullPockets(hullVertices, depthThreshold);
   }

   public ConcaveHullPocket findFirstConcaveHullPocket()
   {
      return findFirstConcaveHullPocket(0);
   }

   public ConcaveHullPocket findFirstConcaveHullPocket(int startIndex)
   {
      return ConcaveHullTools.findFirstConcaveHullPocket(hullVertices, startIndex);
   }

   public boolean isConvexAtVertex(int vertexIndex)
   {
      return ConcaveHullTools.isConvexAtVertex(vertexIndex, hullVertices);
   }

   public boolean isAlmostConvexAtVertex(int vertexIndex, double angleTolerance)
   {
      return ConcaveHullTools.isAlmostConvexAtVertex(vertexIndex, angleTolerance, hullVertices);
   }

   public double getAngleFromPreviousEdgeToNextEdge(int vertexIndex)
   {
      return ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(vertexIndex, hullVertices);
   }

   public boolean isHullConvex()
   {
      return ConcaveHullTools.isHullConvex(hullVertices);
   }

   public int getNumberOfVertices()
   {
      return hullVertices.size();
   }

   public List<Point2d> getConcaveHullVertices()
   {
      return hullVertices;
   }

   public Point2d getVertex(int vertexIndex)
   {
      return hullVertices.get(vertexIndex);
   }

   public List<Point3d> toVerticesInWorld(Point3d hullOrigin, Quat4d hullOrientation)
   {
      return PolygonizerTools.toPointsInWorld(hullVertices, hullOrigin, hullOrientation);
   }

   public List<Point3d> toVerticesInWorld(Point3d hullOrigin, Vector3d hullNormal)
   {
      return PolygonizerTools.toPointsInWorld(hullVertices, hullOrigin, hullNormal);
   }

   public List<Point3d> toVerticesInWorld(RigidBodyTransform transformToWorld)
   {
      List<Point3d> vertices3d = toVertices3d(0.0);
      vertices3d.forEach(vertex -> transformToWorld.transform(vertex));
      return vertices3d;
   }

   public List<Point3d> toVertices3d(double zOffset)
   {
      return stream().map(vertex -> new Point3d(vertex.getX(), vertex.getY(), zOffset)).collect(Collectors.toList());
   }

   public boolean epsilonEquals(ConcaveHull other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int vertexIndex = 0; vertexIndex <= getNumberOfVertices(); vertexIndex++)
      {
         if (!hullVertices.get(vertexIndex).epsilonEquals(other.hullVertices.get(vertexIndex), epsilon))
            return false;
      }
      return true;
   }

   public Stream<Point2d> stream()
   {
      return hullVertices.stream();
   }

   @Override
   public Iterator<Point2d> iterator()
   {
      return hullVertices.iterator();
   }

   @Override
   public int hashCode()
   {
      int hashCode = 1;
      for (Point2d vertex : this)
         hashCode = 31 * hashCode + (vertex == null ? 0 : vertex.hashCode());
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConcaveHull)
         return equals((ConcaveHull) object);
      else
         return false;
   }

   public boolean equals(ConcaveHull other)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int vertexIndex = 0; vertexIndex <= getNumberOfVertices(); vertexIndex++)
      {
         if (!hullVertices.get(vertexIndex).equals(other.hullVertices.get(vertexIndex)))
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      return "Size: " + getNumberOfVertices() + "\n" + ConcaveHullTools.vertexListToString(hullVertices);
   }
}
