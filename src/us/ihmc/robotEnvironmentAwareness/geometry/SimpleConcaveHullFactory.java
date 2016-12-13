package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Collections;
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

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2d> pointCloud2d, double edgeLengthThreshold)
   {
      return createConcaveHullCollection(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS, true);
   }

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2d> pointCloud2d, double edgeLengthThreshold,
         boolean removeAllTrianglesWithTwoBorderEdges)
   {
      return createConcaveHullCollection(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS, removeAllTrianglesWithTwoBorderEdges);
   }

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2d> pointCloud2d, double edgeLengthThreshold, int maxNumberOfIterations,
         boolean removeAllTrianglesWithTwoBorderEdges)
   {
      if (pointCloud2d.size() <= 3)
         return new ConcaveHullCollection(pointCloud2d);

      return createConcaveHull(pointCloud2d, edgeLengthThreshold, maxNumberOfIterations, removeAllTrianglesWithTwoBorderEdges).getConcaveHullCollection();
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2d> pointCloud2d, double edgeLengthThreshold)
   {
      return createConcaveHull(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS, true);
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2d> pointCloud2d, double edgeLengthThreshold,
         boolean removeAllTrianglesWithTwoBorderEdges)
   {
      return createConcaveHull(pointCloud2d, edgeLengthThreshold, DEFAULT_MAX_NUMBER_OF_ITERATIONS, removeAllTrianglesWithTwoBorderEdges);
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2d> pointCloud2d, double edgeLengthThreshold, int maxNumberOfIterations,
         boolean removeAllTrianglesWithTwoBorderEdges)
   {
      if (pointCloud2d.size() <= 3)
         return null;

      MultiPoint multiPoint = createMultiPoint(pointCloud2d);
      ConcaveHullFactoryResult result = new ConcaveHullFactoryResult();
      ConcaveHullFactoryIntermediateVariables initialVariables = initializeTriangulation(multiPoint, result);
      List<ConcaveHullFactoryIntermediateVariables> variableList = computeConcaveHullBorderEdgesRecursive(edgeLengthThreshold, maxNumberOfIterations,
            removeAllTrianglesWithTwoBorderEdges, initialVariables);
      result.intermediateVariables.addAll(variableList);

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : result.intermediateVariables)
      {
         Geometry concaveHullGeometry = computeConcaveHullGeometry(intermediateVariables.getBorderEdges());
         if (concaveHullGeometry != null)
         {
            ConcaveHull concaveHull = convertGeometryToConcaveHull(concaveHullGeometry);
            concaveHull.ensureClockwiseOrdering();
            concaveHull.removeSuccessiveDuplicateVertices();
            result.concaveHullCollection.add(concaveHull);
         }
      }

      return result;
   }

   public static MultiPoint createMultiPoint(List<Point2d> pointCloud2d)
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

   /**
    * 
    * @param multiPoint the point cloud from which a concave hull is to be computed.
    * @param concaveHullFactoryResult
    * @return
    */
   @SuppressWarnings("unchecked")
   private static ConcaveHullFactoryIntermediateVariables initializeTriangulation(MultiPoint multiPoint, ConcaveHullFactoryResult concaveHullFactoryResult)
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
      // All the triangles resulting from the triangulation.
      List<QuadEdgeTriangle> allTriangles = concaveHullFactoryResult.allTriangles;
      allTriangles.addAll(QuadEdgeTriangle.createOn(subdivision));

      if (REPORT_TIME)
      {
         System.out.println("Triangulation took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()) + " sec.");
      }

      return computeIntermediateVariables(allTriangles);
   }

   public static ConcaveHullFactoryIntermediateVariables computeIntermediateVariables(List<QuadEdgeTriangle> delaunayTriangles)
   {
      ConcaveHullFactoryIntermediateVariables intermediateVariables = new ConcaveHullFactoryIntermediateVariables();
      // Vertices of the concave hull
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      // Triangles with at least one edge that belongs to the concave hull.
      Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.borderTriangles;
      // The output of this method, the edges defining the concave hull
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = intermediateVariables.sortedByLengthQueue;

      QuadEdge firstBorderEdge = null;

      // Initialize the border triangles, edges, and vertices. The triangulation provides that information.
      for (QuadEdgeTriangle triangle : delaunayTriangles)
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
                  if (firstBorderEdge == null)
                     firstBorderEdge = borderEdge.getPrimary();

                  borderEdges.add(borderEdge);
                  borderVertices.add(borderEdge.orig());
                  borderVertices.add(borderEdge.dest());
                  sortedByLengthQueue.add(new ImmutablePair<>(borderEdge, triangle));
               }
            }
         }
      }

      List<QuadEdge> orderedBorderEdges = intermediateVariables.orderedBorderEdges;
      orderedBorderEdges.add(firstBorderEdge);
      Vertex startVertex = firstBorderEdge.orig();
      Vertex currentDestVertex = firstBorderEdge.dest();
      QuadEdge previousEdge = firstBorderEdge;

      while (!currentDestVertex.equals(startVertex))
      {
         List<QuadEdge> incidentEdges = findEdgesIncidentOnOrigin(previousEdge.sym());
         QuadEdge currentEdge = null;

         for (QuadEdge incidentEdge : incidentEdges)
         {
            if (isBorderEdge(incidentEdge, borderEdges))
            {
               currentEdge = incidentEdge;
               break;
            }
         }

         orderedBorderEdges.add(currentEdge);
         previousEdge = currentEdge;
         currentDestVertex = currentEdge.dest();
      }

      return intermediateVariables;
   }

   /**
    * Gets all edges which are incident on the origin of the given edge.
    * 
    * @param startEdge
    *          the edge to start at
    * @return a List of edges which have their origin at the origin of the given
    *         edge
    */
   public static List<QuadEdge> findEdgesIncidentOnOrigin(QuadEdge startEdge)
   {
      List<QuadEdge> incidentEdges = new ArrayList<>();

      QuadEdge currentIncidentEdge = startEdge;
      do
      {
         incidentEdges.add(currentIncidentEdge);
         currentIncidentEdge = currentIncidentEdge.oNext();
      }
      while (currentIncidentEdge != startEdge);
      incidentEdges.remove(0);

      return incidentEdges;
   }

   /**
    * Computes the border edges {@link QuadEdge} that will define the concave hull.
    * This is an iterative process that starts a first guess of the concave hull, and then each iteration consists "breaking" edges that are too long according to the {@code edgeLengthThreshold}.
    * The algorithm is based on the <a href="https://en.wikipedia.org/wiki/Delaunay_triangulation"> Delaunay triangulation </a>.
    * @param edgeLengthThreshold maximum edge length the concave hull can have.
    * @param maxNumberOfIterations option to limit the maximum number of iterations of this algorithm.
    * @param removeAllTrianglesWithTwoBorderEdges when set to true, any triangle with two border edges with be removed regardless of the edges length. This tends to smoothen the resulting concave hull in general.
    * @param intermediateVariables the set of variables pre-initialized used internally to find the border edges of the concave hull.
    * @return list of new intermediate variables containing the sets of edges defining the concave hull(s).
    */
   private static List<ConcaveHullFactoryIntermediateVariables> computeConcaveHullBorderEdgesRecursive(double edgeLengthThreshold, int maxNumberOfIterations,
         boolean removeAllTrianglesWithTwoBorderEdges, ConcaveHullFactoryIntermediateVariables intermediateVariables)
   {
      // Vertices of the concave hull
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      // The output of this method, the edges defining the concave hull
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      QuadEdgeComparator quadEdgeComparator = intermediateVariables.quadEdgeComparator;
      PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = intermediateVariables.sortedByLengthQueue;

      List<ImmutablePair<QuadEdge, QuadEdgeTriangle>> bakup = new ArrayList<>();

      for (int iteration = 0; iteration < maxNumberOfIterations; iteration++)
      {
         boolean hasRemovedATriangle = false;

         // Find the regular triangle with the longest border edge
         // Regular means that the triangle can be removed without generating a vertex with more than 2 connected border edges.
         int longestEdgeIndex = -1;
         int numberOfBorderEdges = -1;
         QuadEdgeTriangle borderTriangleWithLongestEdge = null;

         while (longestEdgeIndex == -1 && !sortedByLengthQueue.isEmpty())
         {
            ImmutablePair<QuadEdge, QuadEdgeTriangle> entry = sortedByLengthQueue.poll();
            QuadEdge edge = entry.getKey();
            QuadEdgeTriangle triangle = entry.getValue();
            int edgeIndex = triangle.getEdgeIndex(edge);
            double currentEdgeLength = quadEdgeComparator.getEdgeLength(edge);

            numberOfBorderEdges = numberOfBorderEdges(triangle, borderEdges);

            if (removeAllTrianglesWithTwoBorderEdges)
            {
               if (currentEdgeLength < edgeLengthThreshold && numberOfBorderEdges == 1)
               {
                  bakup.add(entry);
                  continue;
               }
            }
            else
            {
               if (currentEdgeLength < edgeLengthThreshold)
               {
                  bakup.add(entry);
                  continue;
               }
            }

            // The triangle is a candidate
            boolean singleBorderEdgeCase = numberOfBorderEdges == 1 && !borderVertices.contains(triangle.getVertex(indexOfVertexOppositeToEdge(edgeIndex)));
            boolean doubleBorderEdgeCase = numberOfBorderEdges == 2;
            if (singleBorderEdgeCase || doubleBorderEdgeCase)
            {
               longestEdgeIndex = edgeIndex;
               borderTriangleWithLongestEdge = triangle;
               break;
            }

            // The entry's triangle is not removable this iteration, but it might later.
            // So put it in a backup that'll be emptied back in the main queue.
            // Not elegant, but couldn't figure out a way to navigate the queue and only remove a specific entry.
            // Note: the PriorityQueue's iterator is not sorted.
            bakup.add(entry);
         }

         sortedByLengthQueue.addAll(bakup);
         bakup.clear();

         if (longestEdgeIndex >= 0)
         {
            if (numberOfBorderEdges == 1)
            {
               removeTriangleWithOneBorderEdge(intermediateVariables, longestEdgeIndex, borderTriangleWithLongestEdge);
            }
            else if (numberOfBorderEdges == 2)
            {
               removeTriangleWithTwoBorderEdges(intermediateVariables, longestEdgeIndex, borderTriangleWithLongestEdge);
            }
            else
            {
               throw new RuntimeException("Cannot handle triangle with " + numberOfBorderEdges + " border edges.");
            }
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
         }
      }

      return Collections.singletonList(intermediateVariables);
   }

   private static void removeTriangleWithOneBorderEdge(ConcaveHullFactoryIntermediateVariables intermediateVariables, int longestEdgeIndex,
         QuadEdgeTriangle borderTriangleWithLongestEdge)
   {
      int afterEdgeIndex = QuadEdgeTriangle.nextIndex(longestEdgeIndex);
      int beforeEdgeIndex = QuadEdgeTriangle.nextIndex(afterEdgeIndex);

      Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.borderTriangles;
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      Set<QuadEdgeTriangle> outerTriangles = intermediateVariables.outerTriangles;
      PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = intermediateVariables.sortedByLengthQueue;

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
   }

   private static void removeTriangleWithTwoBorderEdges(ConcaveHullFactoryIntermediateVariables intermediateVariables, int longestEdgeIndex,
         QuadEdgeTriangle borderTriangleWithLongestEdge)
   {
      Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.borderTriangles;
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      Set<QuadEdgeTriangle> outerTriangles = intermediateVariables.outerTriangles;
      PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = intermediateVariables.sortedByLengthQueue;

      int newBorderEdgeIndex = -1;
      QuadEdge newBorderEdge = null;

      // Remove the triangle, its edges, and one vertex
      borderTriangles.remove(borderTriangleWithLongestEdge);
      for (int i = 0; i < 3; i++)
      {
         QuadEdge edge = borderTriangleWithLongestEdge.getEdge(i);
         if (!isBorderEdge(edge, borderEdges))
         {
            newBorderEdgeIndex = i;
            newBorderEdge = edge.sym(); // This edge becomes a border edge as we remove the triangle
            continue;
         }
         borderEdges.remove(edge);
         borderEdges.remove(edge.sym());
         sortedByLengthMap.remove(new ImmutablePair<QuadEdge, QuadEdgeTriangle>(edge, borderTriangleWithLongestEdge));
      }

      borderVertices.remove(borderTriangleWithLongestEdge.getVertex(indexOfVertexOppositeToEdge(newBorderEdgeIndex)));

      // Get and add the one adjacent triangle
      QuadEdgeTriangle newBorderTriangle = (QuadEdgeTriangle) newBorderEdge.getData();

      borderTriangles.add(newBorderTriangle);
      borderEdges.add(newBorderEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdge, newBorderTriangle));

      outerTriangles.add(borderTriangleWithLongestEdge);
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

   public static ConcaveHull convertGeometryToConcaveHull(Geometry geometry)
   {
      return new ConcaveHull(convertGeometryToPoint2dList(geometry));
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

   public static class ConcaveHullFactoryResult
   {
      private final ConcaveHullCollection concaveHullCollection = new ConcaveHullCollection();
      private final List<QuadEdgeTriangle> allTriangles = new ArrayList<>();
      private final List<ConcaveHullFactoryIntermediateVariables> intermediateVariables = new ArrayList<>();

      public ConcaveHullFactoryResult()
      {
      }

      /** @return the set of edges defining the resulting concave hull. */
      public ConcaveHullCollection getConcaveHullCollection()
      {
         return concaveHullCollection;
      }

      /** @return all the triangles resulting from the Delaunay triangulation. */
      public List<QuadEdgeTriangle> getAllTriangles()
      {
         return allTriangles;
      }

      /** @return the intermediate variables used internally by the factory to compute a concave hull. */
      public List<ConcaveHullFactoryIntermediateVariables> getIntermediateVariables()
      {
         return intermediateVariables;
      }
   }

   public static class ConcaveHullFactoryIntermediateVariables
   {
      private final Set<Vertex> borderVertices = new HashSet<>();
      private final Set<QuadEdge> borderEdges = new HashSet<>();
      private final List<QuadEdge> orderedBorderEdges = new ArrayList<>();
      private final Set<QuadEdgeTriangle> borderTriangles = new HashSet<>();
      private final Set<QuadEdgeTriangle> outerTriangles = new HashSet<>();
      private final QuadEdgeComparator quadEdgeComparator = new QuadEdgeComparator();
      private final PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = new PriorityQueue<>(quadEdgeComparator);

      public ConcaveHullFactoryIntermediateVariables()
      {
      }

      /** @return vertices of the concave hull. */
      public Set<Vertex> getBorderVertices()
      {
         return borderVertices;
      }

      public Set<QuadEdge> getBorderEdges()
      {
         return borderEdges;
      }

      public List<QuadEdge> getOrderedBorderEdges()
      {
         return orderedBorderEdges;
      }

      /** @return triangles with at least one edge that belongs to the concave hull. */
      public Set<QuadEdgeTriangle> getBorderTriangles()
      {
         return borderTriangles;
      }

      /** @return former border triangles. Internally used to figure out border edges as triangles are being filtered out. */
      public Set<QuadEdgeTriangle> getOuterTriangles()
      {
         return outerTriangles;
      }

      /** @return sorted queue from longest to shortest edges with their triangle. */
      public PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> getSortedByLengthQueue()
      {
         return sortedByLengthQueue;
      }
   }
}
