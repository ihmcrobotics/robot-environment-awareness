package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.apache.commons.lang3.time.StopWatch;

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
public class SimpleConcaveHullFactory
{
   private static final boolean VERBOSE = false;
   private static final boolean REPORT_TIME = false;
   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

   private final GeometryFactory geometryFactory = new GeometryFactory();

   /** Maximum length that each edge of the computed concave hull can have. This is the only parameter. */
   private double edgeLengthThreshold = Double.NaN;

   private int maxNumberOfIterations = 5000;

   /** Coordinates of the set points from which the concave hull is to be computed */
   private MultiPoint multiPoint;

   /** Triangles with at least one edge that belongs to the concave hull. */
   private final Set<QuadEdgeTriangle> borderTriangles = new HashSet<>();
   /** Vertices of the concave hull */
   private final Set<Vertex> borderVertices = new HashSet<>();
   /** Edges of the concave hull */
   private final Set<QuadEdge> borderEdges = new HashSet<>();
   /** The concave hull represented as a {@link Geometry} */
   private Geometry concaveHullGeometry = null;

   public SimpleConcaveHullFactory()
   {
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void setEdgeLengthThreshold(double edgeLengthThreshold)
   {
      this.edgeLengthThreshold = edgeLengthThreshold;
   }

   public void setPointCloud(List<Point2d> newPointCloud)
   {
      Coordinate[] coordinates = new Coordinate[newPointCloud.size()];

      for (int i = 0; i < newPointCloud.size(); i++)
      {
         Point2d point2d = newPointCloud.get(i);
         coordinates[i] = new Coordinate(point2d.getX(), point2d.getY());
      }

      multiPoint = geometryFactory.createMultiPoint(coordinates);

      borderTriangles.clear();
      borderEdges.clear();
      borderVertices.clear();
      concaveHullGeometry = null;
   }

   public void computeConcaveHull()
   {
      if (multiPoint.getNumPoints() <= 3)
      {
         return;
      }
      computeBorderTriangles();
      computeConcaveHullGeometry();
   }

   private void computeConcaveHullGeometry()
   {
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
      LineString merge = (LineString) lineMerger.getMergedLineStrings().iterator().next();

      if (merge.isRing())
      {
         LinearRing linearRing = new LinearRing(merge.getCoordinateSequence(), geometryFactory);
         Polygon concaveHull = new Polygon(linearRing, null, geometryFactory);
         concaveHullGeometry = concaveHull;
      }
      else
      {
         concaveHullGeometry = merge;
      }
   }

   private void computeBorderTriangles()
   {
      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

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
      // Former border triangles. Used to figure out border edges as triangles are being filtered out.
      Set<QuadEdgeTriangle> outerTriangles = new HashSet<>();

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
               }
            }
         }
      }

      for (int iteration = 0;; iteration++)
      {
         boolean hasRemovedATriangle = false;

         // Find the regular triangle with the longest border edge
         // Regular means that the triangle can be removed without generating a vertex with more than 2 connected border edges.
         double longestEdgeLength = 0.0;
         int longestEdgeIndex = -1;
         QuadEdgeTriangle borderTriangleWithLongestEdge = null;

         for (QuadEdgeTriangle triangle : borderTriangles)
         {
            for (int borderEdgeIndex = 0; borderEdgeIndex < 3; borderEdgeIndex++)
            {
               if (isBorderEdge(triangle, borderEdgeIndex, outerTriangles))
               {
                  QuadEdge borderEdge = triangle.getEdge(borderEdgeIndex);
                  double currentEdgeLength = borderEdge.getLength();

                  // The triangle is a candidate
                  if (currentEdgeLength > longestEdgeLength)
                  {
                     Vertex oppositeVertex = triangle.getVertex(indexOfVertexOppositeToEdge(borderEdgeIndex));

                     // Early prune if:
                     //  1- The triangle has 2 border edges
                     //  2- The vertex opposite to the edge is already a border vertex (connected to 2 border edges).
                     if (numberOfBorderEdges(triangle, borderEdges) < 2 && !borderVertices.contains(oppositeVertex))
                     {
                        longestEdgeLength = currentEdgeLength;
                        longestEdgeIndex = borderEdgeIndex;
                        borderTriangleWithLongestEdge = triangle;
                     }
                  }
               }
            }
         }

         if (longestEdgeLength > edgeLengthThreshold)
         {
            int afterEdgeIndex = QuadEdgeTriangle.nextIndex(longestEdgeIndex);
            int beforeEdgeIndex = QuadEdgeTriangle.nextIndex(afterEdgeIndex);

            // Remove the triangle and add the two inner adjacent triangle
            borderTriangles.remove(borderTriangleWithLongestEdge);
            borderTriangles.add(borderTriangleWithLongestEdge.getAdjacentTriangleAcrossEdge(afterEdgeIndex));
            borderTriangles.add(borderTriangleWithLongestEdge.getAdjacentTriangleAcrossEdge(beforeEdgeIndex));

            // Remove the longest edge and the two others.
            borderEdges.remove(borderTriangleWithLongestEdge.getEdge(longestEdgeIndex));
            borderEdges.remove(borderTriangleWithLongestEdge.getEdge(longestEdgeIndex).sym());
            borderEdges.add(borderTriangleWithLongestEdge.getEdge(afterEdgeIndex));
            borderEdges.add(borderTriangleWithLongestEdge.getEdge(beforeEdgeIndex));

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
   }

   private int numberOfBorderEdges(QuadEdgeTriangle triangle, Set<QuadEdge> borderEdges)
   {
      int numberOfBorderEdges = 0;
      for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
      {
         QuadEdge edge = triangle.getEdge(edgeIndex);
         // Need to check the dual of the edge too (edge != edge.sym())
         if (borderEdges.contains(edge) || borderEdges.contains(edge.sym()))
            numberOfBorderEdges++;
      }
      return numberOfBorderEdges;
   }

   /**
    * Is either a border edge because:
    * <li> there is no adjacent triangle across this edge,
    * <li> the adjacent triangle across this edge has already been removed and thus considered to outside the hull.
    * 
    * @param triangle the triangle to get the edge from
    * @param candidateEdgeIndex the edge on which the check is performed
    * @param outerTriangles {@link Set} of the former border triangles.
    * @return true is {@code triangle.getEdge(candiateEdgeIndex)} is a border edge.
    */
   private static boolean isBorderEdge(QuadEdgeTriangle triangle, int candidateEdgeIndex, Set<QuadEdgeTriangle> outerTriangles)
   {
      return triangle.isBorder(candidateEdgeIndex) || outerTriangles.contains(triangle.getAdjacentTriangleAcrossEdge(candidateEdgeIndex));
   }

   private int indexOfVertexOppositeToEdge(int edgeIndex)
   {
      return QuadEdgeTriangle.nextIndex(QuadEdgeTriangle.nextIndex(edgeIndex));
   }

   public List<Point2d> concaveHullAsListOfPoint2d()
   {
      List<Point2d> concaveHullVertices = new ArrayList<>();
      concaveHullAsPoint2dList(concaveHullVertices);
      return concaveHullVertices;
   }

   public void concaveHullAsPoint2dList(List<Point2d> concaveHullVerticesToPack)
   {
      concaveHullVerticesToPack.clear();
      for (Coordinate vertex : concaveHullGeometry.getCoordinates())
         concaveHullVerticesToPack.add(new Point2d(vertex.x, vertex.y));
   }

   public List<Point3d> concaveHullAsListOfPoint3d()
   {
      return concaveHullAsListOfPoint3d(0.0);
   }

   public List<Point3d> concaveHullAsListOfPoint3d(double z)
   {
      List<Point3d> vertices = new ArrayList<>();
      for (Coordinate vertex : concaveHullGeometry.getCoordinates())
         vertices.add(new Point3d(vertex.x, vertex.y, z));
      return vertices;
   }
}
