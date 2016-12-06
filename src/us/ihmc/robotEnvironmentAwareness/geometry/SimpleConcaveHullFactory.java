package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;
import java.util.Set;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.lang3.tuple.ImmutablePair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.operation.linemerge.LineMerger;
import com.vividsolutions.jts.triangulate.ConformingDelaunayTriangulationBuilder;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeSubdivision;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.robotics.time.TimeTools;

/**
 * Computes the concave hull of a 2D point cloud based on the paper
 * <a href="http://worboys.org/publications/Duckham%20Worboys%20Galton%20PatRec%20final%202008.pdf">
 *  Efficient generation of simple polygons for characterizing the shape of a set of points in the plane </a>.
 *  <p>
 *  To summarize, it is based on the Delaunay triangulation.
 *  The only parameter for this algorithm is the maximum edge length the concave hull.
 *  The resulting concave hull is simple:
 *  <li> no self intersections,
 *  <li> no holes,
 *  <li> no detached polygons.
 *  
 * @author Sylvain
 *
 */
public abstract class SimpleConcaveHullFactory
{
   private static final boolean VERBOSE = false;
   private static final boolean REPORT_TIME = false;
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 5000;

   public static List<Point2d> createConcaveHullAsPoint2dList(List<Point2d> pointCloud2d, double edgeLengthThreshold)
   {
      return createConcaveHullAsPoint2dList(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS);
   }

   public static List<Point2d> createConcaveHullAsPoint2dList(List<Point2d> pointCloud2d, double edgeLengthThreshold, int maxNumberOfIterations)
   {
      if (pointCloud2d.size() <= 3)
         return pointCloud2d;


      MultiPoint multiPoint = createMultiPoint(pointCloud2d);
      Set<QuadEdge> borderEdges = computeBorderEdges(multiPoint, edgeLengthThreshold, maxNumberOfIterations);
      Geometry concaveHullGeometry = computeConcaveHullGeometry(borderEdges);
      if (concaveHullGeometry == null)
         return new ArrayList<>();
      else
         return convertGeometryToPoint2dList(concaveHullGeometry);
   }

   public static List<Point3d> createConcaveHullAsPoint3dList(List<Point2d> pointCloud2d, double edgeLengthThreshold, double zConstant)
   {
      return createConcaveHullAsPoint3dList(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS, zConstant);
   }

   public static List<Point3d> createConcaveHullAsPoint3dList(List<Point2d> pointCloud2d, double edgeLengthThreshold, int maxNumberOfIterations, double zConstant)
   {
      List<Point2d> concaveHullAsPoint2dList = createConcaveHullAsPoint2dList(pointCloud2d, edgeLengthThreshold, maxNumberOfIterations);
      return convertPoint2dListToPoint3dList(concaveHullAsPoint2dList, zConstant);
   }

   private static MultiPoint createMultiPoint(List<Point2d> pointCloud2d)
   {
      Coordinate[] coordinates = new Coordinate[pointCloud2d.size()];

      for (int i = 0; i < pointCloud2d.size(); i++)
      {
         Point2d point2d = pointCloud2d.get(i);
         coordinates[i] = new Coordinate(point2d.getX(), point2d.getY());
      }

      return new GeometryFactory().createMultiPoint(coordinates);
   }

   private static Geometry computeConcaveHullGeometry(Set<QuadEdge> borderEdges)
   {
      GeometryFactory geometryFactory = new GeometryFactory();

      // concave hull creation
      List<LineString> edges = new ArrayList<LineString>();
      for (QuadEdge edge : borderEdges)
      {
         LineString lineString = edge.toLineSegment().toGeometry(geometryFactory);
         edges.add(lineString);
      }

      // merge
      LineMerger lineMerger = new LineMerger();
      lineMerger.add(edges);
      LineString merge;
      try
      {
         merge = (LineString) lineMerger.getMergedLineStrings().iterator().next();
      }
      catch (NoSuchElementException e)
      {
         // FIXME Not sure yet why would that happen, needs to be debugged and fixed
         return null;
      }

      if (merge.isRing())
      {
         LinearRing linearRing = new LinearRing(merge.getCoordinateSequence(), geometryFactory);
         Polygon concaveHull = new Polygon(linearRing, null, geometryFactory);
         return concaveHull;
      }
      else
      {
         return merge;
      }
   }

   private static Set<QuadEdge> computeBorderEdges(MultiPoint multiPoint, double edgeLengthThreshold, int maxNumberOfIterations)
   {
      StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      // NOTE: The DelaunayTriangulatorBuilder is 30% to 40% faster than the ConformingDelaunayTriangulationBuilder.
      ConformingDelaunayTriangulationBuilder conformingDelaunayTriangulationBuilder = new ConformingDelaunayTriangulationBuilder();
      conformingDelaunayTriangulationBuilder.setSites(multiPoint);
      QuadEdgeSubdivision subdivision = conformingDelaunayTriangulationBuilder.getSubdivision();

      if (REPORT_TIME)
      {
         System.out.println("Triangulation took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()) + " sec.");
      }

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      // All the triangles resulting from the triangulation.
      @SuppressWarnings("unchecked")
      List<QuadEdgeTriangle> allTriangles = QuadEdgeTriangle.createOn(subdivision);
      // Vertices of the concave hull
      Set<Vertex> borderVertices = new HashSet<>();
      // Triangles with at least one edge that belongs to the concave hull.
      Set<QuadEdgeTriangle> borderTriangles = new HashSet<>();
      // Former border triangles. Used to figure out border edges as triangles are being filtered out.
      Set<QuadEdgeTriangle> outerTriangles = new HashSet<>();
      // The output of this method, the edges defining the concave hull
      Set<QuadEdge> borderEdges = new HashSet<>();
      QuadEdgeComparator quadEdgeComparator = new QuadEdgeComparator();
      PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = new PriorityQueue<>(quadEdgeComparator);

      // Initialize the border triangles, edges, and vertices. The triangulation provides that information.
      for (QuadEdgeTriangle triangle : allTriangles)
      {
         // Direct result from the triangulation
         if (triangle.isBorder())
         {
            borderTriangles.add(triangle);
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
               // Direct result from the triangulation
               if (triangle.isBorder(edgeIndex))
               {
                  QuadEdge borderEdge = triangle.getEdge(edgeIndex);
                  borderEdges.add(borderEdge);
                  borderVertices.add(borderEdge.orig());
                  borderVertices.add(borderEdge.dest());
                  sortedByLengthMap.add(new ImmutablePair<>(borderEdge, triangle));
               }
            }
         }
      }

      for (int iteration = 0;; iteration++)
      {
         boolean hasRemovedATriangle = false;

         // Find the regular triangle with the longest border edge
         // Regular means that the triangle can be removed without generating a vertex with more than 2 connected border edges.
         int longestEdgeIndex = -1;
         QuadEdgeTriangle borderTriangleWithLongestEdge = null;

         while (longestEdgeIndex == -1 && !sortedByLengthMap.isEmpty())
         {
            ImmutablePair<QuadEdge, QuadEdgeTriangle> entry = sortedByLengthMap.poll();
            QuadEdge edge = entry.getKey();
            QuadEdgeTriangle triangle = entry.getValue();
            int edgeIndex = triangle.getEdgeIndex(edge);
            double currentEdgeLength = quadEdgeComparator.getEdgeLength(edge);

            if (currentEdgeLength < edgeLengthThreshold)
               break;

            // The triangle is a candidate
            if (numberOfBorderEdges(triangle, borderEdges) < 2)
            {
               if (!borderVertices.contains(triangle.getVertex(indexOfVertexOppositeToEdge(edgeIndex))))
               {
                  longestEdgeIndex = edgeIndex;
                  borderTriangleWithLongestEdge = triangle;
               }
            }
         }

         if (longestEdgeIndex >= 0)
         {
            int afterEdgeIndex = QuadEdgeTriangle.nextIndex(longestEdgeIndex);
            int beforeEdgeIndex = QuadEdgeTriangle.nextIndex(afterEdgeIndex);

            // Remove the triangle and its edge
            borderTriangles.remove(borderTriangleWithLongestEdge);
            borderEdges.remove(borderTriangleWithLongestEdge.getEdge(longestEdgeIndex));
            borderEdges.remove(borderTriangleWithLongestEdge.getEdge(longestEdgeIndex).sym());

            // Get and add the two adjacent triangles
            QuadEdge afterEdge = borderTriangleWithLongestEdge.getEdge(afterEdgeIndex).sym();
            QuadEdgeTriangle afterTriangle = (QuadEdgeTriangle) afterEdge.getData();
            QuadEdge beforeEdge = borderTriangleWithLongestEdge.getEdge(beforeEdgeIndex).sym();
            QuadEdgeTriangle beforeTriangle = (QuadEdgeTriangle) beforeEdge.getData();

            borderTriangles.add(afterTriangle);
            borderEdges.add(afterEdge);
            sortedByLengthMap.add(new ImmutablePair<>(afterEdge, afterTriangle));

            borderTriangles.add(beforeTriangle);
            borderEdges.add(beforeEdge);
            sortedByLengthMap.add(new ImmutablePair<>(beforeEdge, beforeTriangle));

            // Add the vertex opposite of the removed edge. Its index is the same as beforeEdgeIndex
            borderVertices.add(borderTriangleWithLongestEdge.getVertex(beforeEdgeIndex));

            outerTriangles.add(borderTriangleWithLongestEdge);
            hasRemovedATriangle = true;
         }

         if (!hasRemovedATriangle)
         {
            if (VERBOSE)
               System.out.println("Done, number of iterations: " + iteration);
            break;
         }
         else if (iteration >= maxNumberOfIterations)
         {
            if (VERBOSE)
               System.out.println("Reached max number of iterations");
            break;
         }
      }

      if (REPORT_TIME)
      {
         stopWatch.stop();
         System.out.println("Concave hull computation took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()) + " sec.");
      }

      return borderEdges;
   }

   private static int numberOfBorderEdges(QuadEdgeTriangle triangle, Set<QuadEdge> borderEdges)
   {
      int numberOfBorderEdges = 0;
      for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
      {
         QuadEdge edge = triangle.getEdge(edgeIndex);
         // Need to check the dual of the edge too (edge != edge.sym())
         if (isBorderEdge(edge, borderEdges))
            numberOfBorderEdges++;
      }
      return numberOfBorderEdges;
   }

   private static boolean isBorderEdge(QuadEdge edge, Set<QuadEdge> borderEdges)
   {
      return borderEdges.contains(edge) || borderEdges.contains(edge.sym());
   }

   private static int indexOfVertexOppositeToEdge(int edgeIndex)
   {
      if (edgeIndex < 0 || edgeIndex > 2)
         throw new RuntimeException("Bad edge index: " + edgeIndex);
      return QuadEdgeTriangle.nextIndex(QuadEdgeTriangle.nextIndex(edgeIndex));
   }

   public static List<Point2d> convertGeometryToPoint2dList(Geometry geometry)
   {
      List<Point2d> geometryVertices = new ArrayList<>();
      for (Coordinate vertex : geometry.getCoordinates())
         geometryVertices.add(new Point2d(vertex.x, vertex.y));
      return geometryVertices;
   }

   public static List<Point3d> convertGeometryToPoint3dList(Geometry geometry, double zConstant)
   {
      List<Point3d> geometryVertices = new ArrayList<>();
      for (Coordinate vertex : geometry.getCoordinates())
         geometryVertices.add(new Point3d(vertex.x, vertex.y, zConstant));
      return geometryVertices;
   }

   public static List<Point3d> convertPoint2dListToPoint3dList(List<Point2d> point2ds, double zConstant)
   {
      List<Point3d> point3ds = new ArrayList<>();
      for (Point2d point2d : point2ds)
         point3ds.add(new Point3d(point2d.x, point2d.y, zConstant));
      return point3ds;
   }

   private static class QuadEdgeComparator implements Comparator<ImmutablePair<QuadEdge, QuadEdgeTriangle>>
   {
      private final Map<QuadEdge, Double> map = new HashMap<>();

      @Override
      public int compare(ImmutablePair<QuadEdge, QuadEdgeTriangle> pair1, ImmutablePair<QuadEdge, QuadEdgeTriangle> pair2)
      {
         double length1 = getEdgeLength(pair1.getKey());
         double length2 = getEdgeLength(pair2.getKey());
         if (length1 < length2)
            return 1;
         else if (length1 == length2)
            return 0;
         else
            return -1;
      }

      private double getEdgeLength(QuadEdge edge)
      {
         Double length = map.get(edge);
         if (length == null)
         {
            length = edge.getLength();
            map.put(edge, length);
            map.put(edge.sym(), length);
         }
         return length;
      }
   }
}
