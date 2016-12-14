package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.triangulate.ConformingDelaunayTriangulationBuilder;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeSubdivision;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import us.ihmc.robotics.lists.ListWrappingIndexTools;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;

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

   public static ConcaveHullCollection createConcaveHullCollection(List<Point2d> pointCloud2d, ConcaveHullFactoryParameters parameters)
   {
      if (pointCloud2d.size() <= 3)
         return new ConcaveHullCollection(pointCloud2d);

      return createConcaveHull(pointCloud2d, parameters).getConcaveHullCollection();
   }

   public static ConcaveHullFactoryResult createConcaveHull(List<Point2d> pointCloud2d, ConcaveHullFactoryParameters parameters)
   {
      if (pointCloud2d.size() <= 3)
         return null;

      MultiPoint multiPoint = createMultiPoint(pointCloud2d);
      ConcaveHullFactoryResult result = new ConcaveHullFactoryResult();
      ConcaveHullFactoryIntermediateVariables initialVariables = initializeTriangulation(multiPoint, result);
      List<ConcaveHullFactoryIntermediateVariables> variableList = computeConcaveHullBorderEdgesRecursive(parameters, initialVariables);
      result.intermediateVariables.addAll(variableList);

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : result.intermediateVariables)
      {
         ConcaveHull concaveHull = computeConcaveHull(intermediateVariables.getOrderedBorderEdges());
         if (concaveHull != null)
         {
            concaveHull.ensureClockwiseOrdering();
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

   private static ConcaveHull computeConcaveHull(List<QuadEdge> orderedBorderEdges)
   {
      List<Point2d> orderedConcaveHullVertices = orderedBorderEdges.stream()
                                                                   .map(QuadEdge::orig)
                                                                   .map(vertex -> new Point2d(vertex.getX(), vertex.getY()))
                                                                   .collect(Collectors.toList());
      return new ConcaveHull(orderedConcaveHullVertices);
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
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = intermediateVariables.sortedByLengthQueue;

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

      while (true)
      {
         QuadEdge currentEdge = null;
         QuadEdge currentIncidentEdge = previousEdge.dNext();

         while (currentIncidentEdge != previousEdge)
         {
            if (isBorderEdge(currentIncidentEdge, borderEdges))
            {
               currentEdge = currentIncidentEdge.sym();
               break;
            }
            currentIncidentEdge = currentIncidentEdge.dNext();
         }

         if (currentDestVertex.equals(startVertex))
            break;

         orderedBorderEdges.add(currentEdge);
         previousEdge = currentEdge;
         currentDestVertex = currentEdge.dest();
      }

      return intermediateVariables;
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
   private static List<ConcaveHullFactoryIntermediateVariables> computeConcaveHullBorderEdgesRecursive(ConcaveHullFactoryParameters parameters, ConcaveHullFactoryIntermediateVariables intermediateVariables)
   {
      return computeConcaveHullBorderEdgesRecursive(parameters, intermediateVariables, new MutableInt(0));
   }

   private enum Case
   {
      ONE_BORDER_EDGE_TWO_BORDER_VERTICES, TWO_BORDER_EDGES_THREE_BORDER_VERTICES, ONE_BORDER_EDGES_THREE_BORDER_VERTICES, KEEP_TRIANGLE, THREE_BORDER_EDGES_THREE_BORDER_VERTICES;

      public boolean shouldKeepTriangle()
      {
         switch (this)
         {
         case KEEP_TRIANGLE:
            return true;
         case ONE_BORDER_EDGE_TWO_BORDER_VERTICES:
         case TWO_BORDER_EDGES_THREE_BORDER_VERTICES:
         case ONE_BORDER_EDGES_THREE_BORDER_VERTICES:
         case THREE_BORDER_EDGES_THREE_BORDER_VERTICES:
            return false;
         default:
            throw new RuntimeException("Unknown case: " + this);
         }
      }
   };

   @SuppressWarnings("unchecked")
   public static Case determineCase(Pair<QuadEdge, QuadEdgeTriangle> candidatePair, ConcaveHullFactoryParameters parameters, ConcaveHullFactoryIntermediateVariables intermediateVariables)
   {
      QuadEdgeComparator quadEdgeComparator = intermediateVariables.quadEdgeComparator;
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;

      QuadEdge candidateEdge = candidatePair.getLeft();
      QuadEdgeTriangle candidateTriangle = candidatePair.getRight();
      double edgeLength = quadEdgeComparator.getEdgeLength(candidateEdge);
      boolean isEdgeTooLong = edgeLength >= parameters.getEdgeLengthThreshold();
      int numberOfBorderVertices = numberOfBorderVertices(candidateTriangle, borderVertices);
      int numberOfBorderEdges = numberOfBorderEdges(candidateTriangle, borderEdges);

      if (numberOfBorderVertices == 2)
      {
         if (numberOfBorderEdges != 1)
            throw new RuntimeException("Triangle should have one border edge, but has: " + numberOfBorderEdges);

         return isEdgeTooLong ? Case.ONE_BORDER_EDGE_TWO_BORDER_VERTICES : Case.KEEP_TRIANGLE;
      }

      if (numberOfBorderEdges == 2)
      {
         if (numberOfBorderVertices != 3)
            throw new RuntimeException("Triangle should have three border vertices, but has: " + numberOfBorderVertices);

         int vertexIndexOppositeToCandidateEdge = indexOfVertexOppositeToEdge(candidateTriangle.getEdgeIndex(candidateEdge));
         List<QuadEdgeTriangle> adjacentTrianglesToVertex = candidateTriangle.getTrianglesAdjacentToVertex(vertexIndexOppositeToCandidateEdge);

         boolean atLeastOneAdjacentBorderTriangle = adjacentTrianglesToVertex.stream()
                                                                             .filter(triangle -> isBorderTriangle(triangle, borderEdges))
                                                                             .findFirst()
                                                                             .isPresent();
         if (atLeastOneAdjacentBorderTriangle)
         {
            PrintTools.warn("Messed up hull, skipping triangle");
            return Case.KEEP_TRIANGLE;
         }

         if (parameters.doRemoveAllTrianglesWithTwoBorderEdges() || isEdgeTooLong)
            return Case.TWO_BORDER_EDGES_THREE_BORDER_VERTICES;
         else
            return Case.KEEP_TRIANGLE;
      }

      if (!parameters.isSplittingConcaveHullAllowed())
         return Case.KEEP_TRIANGLE;

      if (numberOfBorderEdges == 1)
         return Case.ONE_BORDER_EDGES_THREE_BORDER_VERTICES;
      else
         return Case.THREE_BORDER_EDGES_THREE_BORDER_VERTICES;
   }

   private static List<ConcaveHullFactoryIntermediateVariables> computeConcaveHullBorderEdgesRecursive(ConcaveHullFactoryParameters parameters, ConcaveHullFactoryIntermediateVariables intermediateVariables, MutableInt currentIteration)
   {
      if (currentIteration.intValue() >= parameters.getMaxNumberOfIterations())
      {
         if (VERBOSE)
            System.out.println("Reached max number of iterations");
         return Collections.singletonList(intermediateVariables);
      }

      if (intermediateVariables == null)
         return Collections.emptyList();

      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = intermediateVariables.sortedByLengthQueue;

      List<Pair<QuadEdge, QuadEdgeTriangle>> bakup = new ArrayList<>();

      Case currentCase = Case.KEEP_TRIANGLE;
      Pair<QuadEdge, QuadEdgeTriangle> candidateEntry = null;

      while (!sortedByLengthQueue.isEmpty())
      {
         candidateEntry = sortedByLengthQueue.poll();
         currentCase = determineCase(candidateEntry, parameters, intermediateVariables);
         if (currentCase.shouldKeepTriangle())
         {
            // The entry's triangle is not removable this iteration, but it might later.
            // So put it in a backup that'll be emptied back in the main queue.
            // Not elegant, but couldn't figure out a way to navigate the queue and only remove a specific entry.
            // Note: the PriorityQueue's iterator is not sorted.
            bakup.add(candidateEntry);
            continue;
         }
         else
         {
            break;
         }
      }

      sortedByLengthQueue.addAll(bakup);
      bakup.clear();

      switch (currentCase)
      {
      case ONE_BORDER_EDGE_TWO_BORDER_VERTICES:
         removeTriangleWithOneBorderEdge(intermediateVariables, candidateEntry);
         currentIteration.increment();
         return computeConcaveHullBorderEdgesRecursive(parameters, intermediateVariables, currentIteration);
      case TWO_BORDER_EDGES_THREE_BORDER_VERTICES:
         removeTriangleWithTwoBorderEdges(intermediateVariables, candidateEntry);
         currentIteration.increment();
         return computeConcaveHullBorderEdgesRecursive(parameters, intermediateVariables, currentIteration);
      case ONE_BORDER_EDGES_THREE_BORDER_VERTICES:
      {
         Pair<ConcaveHullFactoryIntermediateVariables, ConcaveHullFactoryIntermediateVariables> subVariables = removeTriangleAndDivideHull(intermediateVariables, candidateEntry);
         currentIteration.increment();
         List<ConcaveHullFactoryIntermediateVariables> left = computeConcaveHullBorderEdgesRecursive(parameters, subVariables.getLeft(), currentIteration);
         currentIteration.increment();
         List<ConcaveHullFactoryIntermediateVariables> right = computeConcaveHullBorderEdgesRecursive(parameters, subVariables.getRight(), currentIteration);
         ArrayList<ConcaveHullFactoryIntermediateVariables> result = new ArrayList<>();
         result.addAll(left);
         result.addAll(right);
         return result;
      }
      case THREE_BORDER_EDGES_THREE_BORDER_VERTICES:
         return Collections.emptyList();
      case KEEP_TRIANGLE:
         if (VERBOSE)
            System.out.println("Done, number of iterations: " + currentIteration.intValue());
         return Collections.singletonList(intermediateVariables);
      default:
         throw new RuntimeException("Unknown case: " + currentCase);
      }
   }

   private static void removeTriangleWithOneBorderEdge(ConcaveHullFactoryIntermediateVariables intermediateVariables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      QuadEdgeTriangle borderTriangleToRemove = entryToRemove.getRight();
      QuadEdge edgeToRemove = entryToRemove.getLeft();

      int indexOfTriangleEdgeToRemove = borderTriangleToRemove.getEdgeIndex(edgeToRemove);
      int indexAfterRemovedEdge = QuadEdgeTriangle.nextIndex(indexOfTriangleEdgeToRemove);
      int indexBeforeRemovedEdge = QuadEdgeTriangle.nextIndex(indexAfterRemovedEdge);

      Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.borderTriangles;
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = intermediateVariables.sortedByLengthQueue;

      // Remove the triangle and its edge
      borderTriangles.remove(borderTriangleToRemove);
      borderEdges.remove(edgeToRemove);
      borderEdges.remove(edgeToRemove.sym());

      // Get and add the two adjacent triangles
      QuadEdge newBorderEdgeAfterRemovedEdge = borderTriangleToRemove.getEdge(indexAfterRemovedEdge).sym();
      QuadEdgeTriangle newBorderTriangleAfterRemovedTriangle = (QuadEdgeTriangle) newBorderEdgeAfterRemovedEdge.getData();
      QuadEdge newBorderEdgeBeforeRemovedEdge = borderTriangleToRemove.getEdge(indexBeforeRemovedEdge).sym();
      QuadEdgeTriangle newBorderTriangleBeforeRemovedTriangle = (QuadEdgeTriangle) newBorderEdgeBeforeRemovedEdge.getData();

      borderTriangles.add(newBorderTriangleAfterRemovedTriangle);
      borderEdges.add(newBorderEdgeAfterRemovedEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdgeAfterRemovedEdge, newBorderTriangleAfterRemovedTriangle));

      borderTriangles.add(newBorderTriangleBeforeRemovedTriangle);
      borderEdges.add(newBorderEdgeBeforeRemovedEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdgeBeforeRemovedEdge, newBorderTriangleBeforeRemovedTriangle));

      // Add the vertex opposite of the removed edge. Its index is the same as beforeEdgeIndex
      borderVertices.add(borderTriangleToRemove.getVertex(indexBeforeRemovedEdge));

      List<QuadEdge> orderedBorderEdges = intermediateVariables.orderedBorderEdges;
      replaceOneEdgeWithTwoInOrderedList(orderedBorderEdges, edgeToRemove, newBorderEdgeBeforeRemovedEdge, newBorderEdgeAfterRemovedEdge);
   }

   private static void removeTriangleWithTwoBorderEdges(ConcaveHullFactoryIntermediateVariables intermediateVariables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      QuadEdgeTriangle borderTriangleToRemove = entryToRemove.getRight();

      Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.borderTriangles;
      Set<QuadEdge> borderEdges = intermediateVariables.borderEdges;
      Set<Vertex> borderVertices = intermediateVariables.borderVertices;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthMap = intermediateVariables.sortedByLengthQueue;

      int newBorderEdgeIndex = -1;
      QuadEdge newBorderEdge = null;

      // Remove the triangle, its edges, and one vertex
      borderTriangles.remove(borderTriangleToRemove);
      for (int i = 0; i < 3; i++)
      {
         QuadEdge edge = borderTriangleToRemove.getEdge(i);
         if (!isBorderEdge(edge, borderEdges))
         {
            newBorderEdgeIndex = i;
            newBorderEdge = edge.sym(); // This edge becomes a border edge as we remove the triangle
            continue;
         }
         borderEdges.remove(edge);
         borderEdges.remove(edge.sym());
         sortedByLengthMap.remove(new ImmutablePair<>(edge, borderTriangleToRemove));
      }

      borderVertices.remove(borderTriangleToRemove.getVertex(indexOfVertexOppositeToEdge(newBorderEdgeIndex)));

      // Get and add the one adjacent triangle
      QuadEdgeTriangle newBorderTriangle = (QuadEdgeTriangle) newBorderEdge.getData();

      borderTriangles.add(newBorderTriangle);
      borderEdges.add(newBorderEdge);
      sortedByLengthMap.add(new ImmutablePair<>(newBorderEdge, newBorderTriangle));

      List<QuadEdge> orderedBorderEdges = intermediateVariables.orderedBorderEdges;
      replaceTwoEdgesWithOneInOrderedList(orderedBorderEdges, borderTriangleToRemove.getEdge((newBorderEdgeIndex + 2) % 3), borderTriangleToRemove.getEdge((newBorderEdgeIndex + 1) % 3), newBorderEdge);
   }

   private static Pair<ConcaveHullFactoryIntermediateVariables, ConcaveHullFactoryIntermediateVariables> removeTriangleAndDivideHull(
         ConcaveHullFactoryIntermediateVariables intermediateVariables, Pair<QuadEdge, QuadEdgeTriangle> entryToRemove)
   {
      List<QuadEdge> orderedBorderEdges = intermediateVariables.orderedBorderEdges;
      PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = intermediateVariables.sortedByLengthQueue;

      QuadEdge edgeToRemove = entryToRemove.getLeft();
      QuadEdgeTriangle borderTriangleToRemove = entryToRemove.getRight();
      int indexOfTriangleEdgeToRemove = borderTriangleToRemove.getEdgeIndex(edgeToRemove);
      Vertex vertexOppositeToEdgeToRemove = borderTriangleToRemove.getVertex(indexOfVertexOppositeToEdge(indexOfTriangleEdgeToRemove));

      int orderedIndexOfEdgeToRemove = orderedBorderEdges.indexOf(edgeToRemove);
      if (orderedIndexOfEdgeToRemove == -1)
      {
         edgeToRemove = edgeToRemove.sym();
         orderedIndexOfEdgeToRemove = orderedBorderEdges.indexOf(edgeToRemove);
      }
      if (orderedIndexOfEdgeToRemove == -1)
         throw new RuntimeException("Did not find edge to remove in the ordered border edge list.");

      int firstHullStartIndex = ListWrappingIndexTools.next(orderedIndexOfEdgeToRemove, orderedBorderEdges);
      int firstHullEndIndex = ListWrappingIndexTools.next(firstHullStartIndex, orderedBorderEdges);

      int count = 0;

      while (!doesVertexBelongToQuadEdge(vertexOppositeToEdgeToRemove, orderedBorderEdges.get(firstHullEndIndex)))
      {
         firstHullEndIndex = ListWrappingIndexTools.next(firstHullEndIndex, orderedBorderEdges);
         if (count++ >= orderedBorderEdges.size())
            throw new RuntimeException("Wrapped ");
      }

      int secondHullStartIndex = ListWrappingIndexTools.next(firstHullEndIndex, orderedBorderEdges);
      int secondHullEndIndex = ListWrappingIndexTools.previous(orderedIndexOfEdgeToRemove, orderedBorderEdges);

      int indexOfEdgeToKeep1 = (indexOfTriangleEdgeToRemove + 1) % 3;
      int indexOfEdgeToKeep2 = (indexOfTriangleEdgeToRemove + 2) % 3;
      QuadEdge edgeToKeep1 = borderTriangleToRemove.getEdge(indexOfEdgeToKeep1);
      QuadEdge edgeToKeep2 = borderTriangleToRemove.getEdge(indexOfEdgeToKeep2);

      QuadEdge edgeToAddToFirstHull;
      QuadEdge edgeToAddToSecondHull;

      QuadEdgeTriangle triangleToAddToFirstHull;
      QuadEdgeTriangle triangleToAddToSecondHull;

      if (doesVertexBelongToQuadEdge(orderedBorderEdges.get(firstHullStartIndex).orig(), edgeToKeep1))
      {
         edgeToAddToFirstHull = edgeToKeep1;
         edgeToAddToSecondHull = edgeToKeep2;
         triangleToAddToFirstHull = borderTriangleToRemove.getAdjacentTriangleAcrossEdge(indexOfEdgeToKeep1);
         triangleToAddToSecondHull = borderTriangleToRemove.getAdjacentTriangleAcrossEdge(indexOfEdgeToKeep2);
      }
      else
      {
         edgeToAddToFirstHull = edgeToKeep2;
         edgeToAddToSecondHull = edgeToKeep1;
         triangleToAddToFirstHull = borderTriangleToRemove.getAdjacentTriangleAcrossEdge(indexOfEdgeToKeep2);
         triangleToAddToSecondHull = borderTriangleToRemove.getAdjacentTriangleAcrossEdge(indexOfEdgeToKeep1);
      }

      MutablePair<ConcaveHullFactoryIntermediateVariables, ConcaveHullFactoryIntermediateVariables> result = new MutablePair<>();

      { // Initializing the variable for the hull 1
         ConcaveHullFactoryIntermediateVariables intermediateVariables1 = new ConcaveHullFactoryIntermediateVariables();
         List<QuadEdge> orderedBorderEdges1 = intermediateVariables1.orderedBorderEdges;
         Set<QuadEdge> borderEdges1 = intermediateVariables1.borderEdges;
         Set<QuadEdgeTriangle> borderTriangles1 = intermediateVariables1.borderTriangles;
         Set<Vertex> borderVertices1 = intermediateVariables1.borderVertices;
         PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue1 = intermediateVariables1.sortedByLengthQueue;

         orderedBorderEdges1.addAll(ListWrappingIndexTools.subListInclusive(firstHullStartIndex, firstHullEndIndex, orderedBorderEdges));
         if (edgeToAddToFirstHull.orig() == orderedBorderEdges1.get(orderedBorderEdges1.size() - 1).dest())
            orderedBorderEdges1.add(edgeToAddToFirstHull);
         else
            orderedBorderEdges1.add(edgeToAddToFirstHull.sym());

         if (orderedBorderEdges1.size() > 3)
         {
            checkOrderedBorderEdgeListValid(orderedBorderEdges1);
            borderEdges1.addAll(orderedBorderEdges1);
            borderEdges1.forEach(borderEdge -> borderVertices1.add(borderEdge.orig()));

            sortedByLengthQueue.stream().filter(pair -> isBorderEdge(pair.getLeft(), borderEdges1)).forEach(sortedByLengthQueue1::add);
            sortedByLengthQueue1.add(new ImmutablePair<>(edgeToAddToFirstHull.sym(), triangleToAddToFirstHull));
            sortedByLengthQueue1.forEach(pair -> borderTriangles1.add(pair.getRight()));

            result.setLeft(intermediateVariables1);
         }
         else
         {
            result.setLeft(null);
         }
      }

      { // Initializing the variable for the hull 2
         ConcaveHullFactoryIntermediateVariables intermediateVariables2 = new ConcaveHullFactoryIntermediateVariables();
         List<QuadEdge> orderedBorderEdges2 = intermediateVariables2.orderedBorderEdges;
         Set<QuadEdge> borderEdges2 = intermediateVariables2.borderEdges;
         Set<QuadEdgeTriangle> borderTriangles2 = intermediateVariables2.borderTriangles;
         Set<Vertex> borderVertices2 = intermediateVariables2.borderVertices;
         PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue2 = intermediateVariables2.sortedByLengthQueue;

         orderedBorderEdges2.addAll(ListWrappingIndexTools.subListInclusive(secondHullStartIndex, secondHullEndIndex, orderedBorderEdges));
         if (edgeToAddToSecondHull.orig() == orderedBorderEdges2.get(orderedBorderEdges2.size() - 1).dest())
            orderedBorderEdges2.add(edgeToAddToSecondHull);
         else
            orderedBorderEdges2.add(edgeToAddToSecondHull.sym());

         if (orderedBorderEdges2.size() > 3)
         {
            checkOrderedBorderEdgeListValid(orderedBorderEdges2);
            borderEdges2.addAll(orderedBorderEdges2);
            borderEdges2.forEach(borderEdge -> borderVertices2.add(borderEdge.orig()));

            sortedByLengthQueue.stream().filter(pair -> isBorderEdge(pair.getLeft(), borderEdges2)).forEach(sortedByLengthQueue2::add);
            sortedByLengthQueue2.add(new ImmutablePair<>(edgeToAddToSecondHull.sym(), triangleToAddToSecondHull));
            sortedByLengthQueue2.forEach(pair -> borderTriangles2.add(pair.getRight()));

            result.setRight(intermediateVariables2);
         }
         else
         {
            result.setRight(null);
         }
      }

      return result;
   }

   private static void replaceOneEdgeWithTwoInOrderedList(List<QuadEdge> orderedBorderEdges, QuadEdge edgeToReplace, QuadEdge newEdge1, QuadEdge newEdge2)
   {
      int indexOfEdgeToReplace = orderedBorderEdges.indexOf(edgeToReplace);
      if (indexOfEdgeToReplace == -1)
      {
         edgeToReplace = edgeToReplace.sym();
         indexOfEdgeToReplace = orderedBorderEdges.indexOf(edgeToReplace);
      }
      if (indexOfEdgeToReplace == -1)
         throw new RuntimeException("Did not find edge to remove in the ordered border edge list.");

      Vertex previousVertex = edgeToReplace.orig();
      Vertex nextVertex = edgeToReplace.dest();

      QuadEdge firstEdge;
      QuadEdge secondEdge;
      
      if (doesVertexBelongToQuadEdge(previousVertex, newEdge1))
      {
         firstEdge = newEdge1.orig() == previousVertex ? newEdge1 : newEdge1.sym();
         secondEdge = newEdge2.dest() == nextVertex ? newEdge2 : newEdge2.sym();
         if (!doesVertexBelongToQuadEdge(nextVertex, newEdge2))
            throw new RuntimeException("newEdge2 is not connected to nextVertex.");
      }
      else if (doesVertexBelongToQuadEdge(previousVertex, newEdge2))
      {
         firstEdge = newEdge2.orig() == previousVertex ? newEdge2 : newEdge2.sym();
         secondEdge = newEdge1.dest() == nextVertex ? newEdge1 : newEdge1.sym();
         if (!doesVertexBelongToQuadEdge(nextVertex, newEdge1))
            throw new RuntimeException("newEdge1 is not connected to nextVertex.");
      }
      else
      {
         throw new RuntimeException("newEdge1 is not connected to either previousVertex or nextVertex.");
      }

      orderedBorderEdges.set(indexOfEdgeToReplace, firstEdge);
      orderedBorderEdges.add(indexOfEdgeToReplace + 1, secondEdge);
      checkOrderedBorderEdgeListValid(orderedBorderEdges);
   }

   private static void replaceTwoEdgesWithOneInOrderedList(List<QuadEdge> orderedBorderEdges, QuadEdge edgeToReplace1, QuadEdge edgeToReplace2, QuadEdge newEdge)
   {
      int firstEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace1);
      if (firstEdgeIndex == -1)
      {
         edgeToReplace1 = edgeToReplace1.sym();
         firstEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace1);
      }
      if (firstEdgeIndex == -1)
         throw new RuntimeException("Did not find the first edge to remove in the ordered border edge list.");
      int secondEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace2);
      if (secondEdgeIndex == -1)
      {
         edgeToReplace2 = edgeToReplace2.sym();
         secondEdgeIndex = orderedBorderEdges.indexOf(edgeToReplace2);
      }
      if (secondEdgeIndex == -1)
         throw new RuntimeException("Did not find the second edge to remove in the ordered border edge list.");

      if (ListWrappingIndexTools.minDistanceExclusive(firstEdgeIndex, secondEdgeIndex, orderedBorderEdges) != 0)
         throw new RuntimeException("The two edges are not connected");

      Vertex previousVertex;
      Vertex nextVertex;

      if (ListWrappingIndexTools.previous(secondEdgeIndex, orderedBorderEdges) == firstEdgeIndex)
      {
         previousVertex = edgeToReplace1.orig();
         nextVertex = edgeToReplace2.dest();
      }
      else
      {
         previousVertex = edgeToReplace2.orig();
         nextVertex = edgeToReplace1.dest();
      }

      QuadEdge edgeToInsert = newEdge.orig() == previousVertex ? newEdge : newEdge.sym();
      if (edgeToInsert.orig() != previousVertex || edgeToInsert.dest() != nextVertex)
         throw new RuntimeException("The new edge to insert is not properly connected to the list.");

      orderedBorderEdges.set(firstEdgeIndex, edgeToInsert);
      orderedBorderEdges.remove(secondEdgeIndex);
      checkOrderedBorderEdgeListValid(orderedBorderEdges);
   }

   private static boolean doesVertexBelongToQuadEdge(Vertex vertex, QuadEdge edge)
   {
      return edge.orig() == vertex || edge.dest() == vertex;
   }

   private static void checkOrderedBorderEdgeListValid(List<QuadEdge> orderedBorderEdges)
   {
      for (int edgeIndex = 0; edgeIndex < orderedBorderEdges.size(); edgeIndex++)
      {
         Vertex currentDest = orderedBorderEdges.get(edgeIndex).dest();
         Vertex nextOrig = ListWrappingIndexTools.getNext(edgeIndex, orderedBorderEdges).orig();

         if (currentDest != nextOrig)
            throw new RuntimeException("Ordered border edge list is corrupted.");
      }
   }

   private static int numberOfBorderEdges(QuadEdgeTriangle triangle, Set<QuadEdge> borderEdges)
   {
      int numberOfBorderEdges = 0;
      for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
      {
         QuadEdge edge = triangle.getEdge(edgeIndex);
         // Need to check the opposite of the edge too (edge != edge.sym())
         if (isBorderEdge(edge, borderEdges))
            numberOfBorderEdges++;
      }
      return numberOfBorderEdges;
   }

   private static int numberOfBorderVertices(QuadEdgeTriangle triangle, Set<Vertex> borderVertices)
   {
      int numberOfBorderVertices = 0;
      for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
      {
         Vertex vertex = triangle.getVertex(vertexIndex);
         if (borderVertices.contains(vertex))
            numberOfBorderVertices++;
      }
      return numberOfBorderVertices;
   }

   private static boolean isBorderEdge(QuadEdge edge, Set<QuadEdge> borderEdges)
   {
      return borderEdges.contains(edge) || borderEdges.contains(edge.sym());
   }

   private static boolean isBorderTriangle(QuadEdgeTriangle triangle, Set<QuadEdge> borderEdges)
   {
      return Arrays.stream(triangle.getEdges())
                   .filter(edge -> isBorderEdge(edge, borderEdges))
                   .findFirst()
                   .isPresent();
   }

   private static int indexOfVertexOppositeToEdge(int edgeIndex)
   {
      if (edgeIndex < 0 || edgeIndex > 2)
         throw new RuntimeException("Bad edge index: " + edgeIndex);
      return QuadEdgeTriangle.nextIndex(QuadEdgeTriangle.nextIndex(edgeIndex));
   }

   private static class QuadEdgeComparator implements Comparator<Pair<QuadEdge, QuadEdgeTriangle>>
   {
      private final Map<QuadEdge, Double> map = new HashMap<>();

      @Override
      public int compare(Pair<QuadEdge, QuadEdgeTriangle> pair1, Pair<QuadEdge, QuadEdgeTriangle> pair2)
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
      private final QuadEdgeComparator quadEdgeComparator = new QuadEdgeComparator();
      private final PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> sortedByLengthQueue = new PriorityQueue<>(quadEdgeComparator);

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

      /** @return sorted queue from longest to shortest edges with their triangle. */
      public PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> getSortedByLengthQueue()
      {
         return sortedByLengthQueue;
      }
   }
}
