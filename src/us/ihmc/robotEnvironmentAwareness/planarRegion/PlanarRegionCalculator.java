package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import javax.vecmath.Vector3d;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.iterators.OcTreeIterable;
import us.ihmc.octoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.rules.interfaces.IteratorSelectionRule;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;
import us.ihmc.robotEnvironmentAwareness.exception.PlanarRegionSegmentationException;

public class PlanarRegionCalculator
{
   private final Set<NormalOcTreeNode> allRegionNodes = new HashSet<>();
   private List<PlanarRegion> planarRegions = new ArrayList<>();
   private final List<NormalOcTreeNode> nodesWithoutRegion = new ArrayList<>();
   private final Random random = new Random(234324L);

   public void compute(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      allRegionNodes.clear();

      planarRegions.stream().forEach(region -> removeBadNodesFromRegion(boundingBox, parameters, region));
      planarRegions.stream().forEach(region -> region.nodeStream().forEach(allRegionNodes::add));
      planarRegions.stream().forEach(region -> growPlanarRegion(root, region, boundingBox, parameters));
      planarRegions.parallelStream().forEach(PlanarRegionCalculator::flipNormalOfOutliers);
      
      Set<NormalOcTreeNode> nodeSet = new HashSet<>();
      nodeSet.clear();
      new OcTreeIterable<>(root, leafInBoundingBoxWithNormalSetRule(boundingBox)).forEach(nodeSet::add);
      nodeSet.removeAll(allRegionNodes);

      nodesWithoutRegion.clear();
      nodesWithoutRegion.addAll(nodeSet);

      planarRegions.addAll(searchNewPlanarRegions(root, boundingBox, parameters, random));
      planarRegions = mergePlanarRegionsIfPossible(root, planarRegions, parameters);
   }

   public void removeDeadNodes()
   {
      planarRegions.stream().forEach(region -> removeDeadNodesFromRegion(region));
   }

   public List<PlanarRegion> getPlanarRegions()
   {
      return planarRegions;
   }

   public void clear()
   {
      planarRegions.clear();
   }

   private IteratorSelectionRule<NormalOcTreeNode> leafInBoundingBoxWithNormalSetRule(OcTreeBoundingBoxInterface boundingBox)
   {
      IteratorSelectionRule<NormalOcTreeNode> isNormalSetRule = (node, maxDepth) -> node.isNormalSet();
      return OcTreeIteratorFactory.multipleRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(boundingBox), isNormalSetRule);
   }

   public static void flipNormalOfOutliers(PlanarRegion planarRegion)
   {
      Vector3d regionNormal = planarRegion.getNormal();
      int numberOfNormalsFlipped = (int) planarRegion.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).count();
      int numberOfNormalsNotFlipped = planarRegion.getNumberOfNodes() - numberOfNormalsFlipped;

      // The majority of the nodes are flipped => flip the region normal
      if (numberOfNormalsFlipped > numberOfNormalsNotFlipped)
         regionNormal.negate();

      // Flip the nodes that upside down
      planarRegion.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).forEach(NormalOcTreeNode::negateNormal);
   }

   private static boolean isNodeNormalFlipped(NormalOcTreeNode node, Vector3d referenceNormal)
   {
      return node.getNormalX() * referenceNormal.getX() + node.getNormalY() * referenceNormal.getY() + node.getNormalZ() * referenceNormal.getZ() < 0.0;
   }

   public static List<PlanarRegion> mergePlanarRegionsIfPossible(NormalOcTreeNode root, List<PlanarRegion> inputRegions, PlanarRegionSegmentationParameters parameters)
   {
      List<PlanarRegion> mergedRegions = new ArrayList<>();
      while (!inputRegions.isEmpty())
      {
         PlanarRegion candidateForMergeOtherRegions = inputRegions.get(0);
         Map<Boolean, List<PlanarRegion>> mergeableAndNonMergeableGroups = inputRegions.subList(1, inputRegions.size()).parallelStream()
                     // Group each region according to the result of areRegionsMergeable.
                    .collect(Collectors.groupingBy(other -> areRegionsMergeable(root, candidateForMergeOtherRegions, other, parameters)));

         // Merge all the mergeable regions onto the candidate.
         mergeableAndNonMergeableGroups.getOrDefault(true, Collections.emptyList()).forEach(candidateForMergeOtherRegions::addNodesFromOtherRegion);
         // All non mergeable regions, used for the next iteration.
         inputRegions = mergeableAndNonMergeableGroups.getOrDefault(false, Collections.emptyList());
         // We're done with candidate, put it in the output list.
         mergedRegions.add(candidateForMergeOtherRegions);
      }
      return mergedRegions;
   }

   public static boolean areRegionsMergeable(NormalOcTreeNode root, PlanarRegion currentRegion, PlanarRegion potentialRegionToMerge,
         PlanarRegionSegmentationParameters parameters)
   {
      if (currentRegion == potentialRegionToMerge)
         throw new PlanarRegionSegmentationException("Problem Houston.");

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      if (currentRegion.absoluteOrthogonalDistance(potentialRegionToMerge.getOrigin()) > maxDistanceFromPlane)
         return false;

      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());

      if (currentRegion.absoluteDot(potentialRegionToMerge) < dotThreshold)
         return false;

      double searchRadius = parameters.getSearchRadius();
      PlanarRegion regionToNavigate;
      PlanarRegion otherRegion;

      if (potentialRegionToMerge.getNumberOfNodes() < currentRegion.getNumberOfNodes())
      {
         regionToNavigate = potentialRegionToMerge;
         otherRegion = currentRegion;
      }
      else
      {
         regionToNavigate = currentRegion;
         otherRegion = potentialRegionToMerge;
      }

      return regionToNavigate.nodeStream()
                             .filter(node -> isNodeInOtherRegionNeighborhood(root, node, otherRegion, searchRadius))
                             .findFirst()
                             .isPresent();
   }

   public static boolean isNodeInOtherRegionNeighborhood(NormalOcTreeNode root, NormalOcTreeNode nodeFromOneRegion, PlanarRegion otherRegion, double searchRadius)
   {
      MutableBoolean foundNeighborFromOtherRegion = new MutableBoolean(false);

      NeighborActionRule<NormalOcTreeNode> actionRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (otherRegion.containsNode(node))
               foundNeighborFromOtherRegion.setTrue();
         }

         @Override
         public boolean earlyAbort()
         {
            return foundNeighborFromOtherRegion.booleanValue();
         }
      };

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, nodeFromOneRegion, searchRadius, actionRule);
      return foundNeighborFromOtherRegion.booleanValue();
   }

   public List<PlanarRegion> searchNewPlanarRegions(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters, Random random)
   {
      List<PlanarRegion> newPlanarRegions = new ArrayList<>();

      float minNormalQuality = (float) parameters.getMinNormalQuality();

      for (NormalOcTreeNode node : nodesWithoutRegion)
      {
         if (node.getNormalAverageDeviation() > minNormalQuality)
            continue;

         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = createNewPlanarRegion(root, node, regionId, boundingBox, parameters);

         if (planarRegion.getNumberOfNodes() < 10)
            planarRegion.clearRegion();
         else
            newPlanarRegions.add(planarRegion);
      }

      return newPlanarRegions;
   }

   public PlanarRegion createNewPlanarRegion(NormalOcTreeNode root, NormalOcTreeNode seedNode, int regionId, OcTreeBoundingBoxInterface boundingBox,
         PlanarRegionSegmentationParameters parameters)
   {
      PlanarRegion planarRegion = new PlanarRegion(regionId);
      planarRegion.addNode(seedNode);
      growPlanarRegion(root, planarRegion, boundingBox, parameters);
      return planarRegion;
   }

   public void growPlanarRegion(NormalOcTreeNode root, PlanarRegion planarRegion, OcTreeBoundingBoxInterface boundingBox,
         PlanarRegionSegmentationParameters parameters)
   {
      double searchRadius = parameters.getSearchRadius();
      
      Deque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
      Set<NormalOcTreeNode> newSetToExplore = new HashSet<>();

      NeighborActionRule<NormalOcTreeNode> extendSearchRule = neighborNode -> recordCandidatesForRegion(neighborNode, planarRegion, newSetToExplore, boundingBox, parameters);
      planarRegion.nodeParallelStream()
                  .filter(node -> isNodeInBoundingBox(node, boundingBox))
                  .forEach(regionNode -> OcTreeNearestNeighborTools.findRadiusNeighbors(root, regionNode, searchRadius, extendSearchRule));
      nodesToExplore.addAll(newSetToExplore);

      while (!nodesToExplore.isEmpty())
      {
         NormalOcTreeNode currentNode = nodesToExplore.poll();
         if (!planarRegion.addNode(currentNode))
            continue;
         allRegionNodes.add(currentNode);
         newSetToExplore.clear();
         OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, extendSearchRule);
         nodesToExplore.addAll(newSetToExplore);
      }
   }

   public void recordCandidatesForRegion(NormalOcTreeNode neighborNode, PlanarRegion planarRegion, Set<NormalOcTreeNode> newSetToExplore, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      if (allRegionNodes.contains(neighborNode))
         return;
      if (!isNodeInBoundingBox(neighborNode, boundingBox))
         return;
      if (!isNodePartOfRegion(neighborNode, planarRegion, parameters.getMaxDistanceFromPlane(), Math.cos(parameters.getMaxAngleFromPlane())))
         return;
      if (!neighborNode.isNormalSet() || !neighborNode.isHitLocationSet())
         return;
      
      newSetToExplore.add(neighborNode);
   }

   private static void removeBadNodesFromRegion(OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters,
         PlanarRegion planarRegion)
   {
      List<NormalOcTreeNode> nodesToRemove = planarRegion.nodeStream()
            .collect(Collectors.groupingBy(node -> isBadNode(node, planarRegion, boundingBox, parameters)))
            .getOrDefault(true, Collections.emptyList());

      planarRegion.removeNodesAndUpdate(nodesToRemove);
   }

   private static void removeDeadNodesFromRegion(PlanarRegion planarRegion)
   {
      List<NormalOcTreeNode> nodesToRemove = planarRegion.nodeStream()
            .collect(Collectors.groupingBy(node -> isNodeDead(node)))
            .getOrDefault(true, Collections.emptyList());

      planarRegion.removeNodesAndUpdate(nodesToRemove);
   }

   private static boolean isNodeInBoundingBox(NormalOcTreeNode node, OcTreeBoundingBoxInterface boundingBox)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(node.getX(), node.getY(), node.getZ());
   }

   private static boolean isBadNode(NormalOcTreeNode node, PlanarRegion planarRegion, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      if (isNodeDead(node))
         return true;

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      return isNodeInBoundingBox(node, boundingBox) && !isNodePartOfRegion(node, planarRegion, maxDistanceFromPlane, dotThreshold);
   }

   private static boolean isNodeDead(NormalOcTreeNode node)
   {
      return !node.isNormalSet();
   }

   public static boolean isNodePartOfRegion(NormalOcTreeNode node, PlanarRegion planarRegion, double maxDistanceFromPlane, double dotThreshold)
   {
      double absoluteOrthogonalDistance = planarRegion.absoluteOrthogonalDistance(node);
      if (absoluteOrthogonalDistance > maxDistanceFromPlane)
         return false;

      double absoluteDot = planarRegion.absoluteDotWithNodeNormal(node);
      return absoluteDot > dotThreshold;
   }
}
