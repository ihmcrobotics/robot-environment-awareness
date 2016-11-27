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

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.rules.interfaces.IteratorSelectionRule;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;
import us.ihmc.robotEnvironmentAwareness.exception.PlanarRegionSegmentationException;

public class OcTreeNodePlanarRegionCalculator
{
   private final Set<NormalOcTreeNode> allRegionNodes = new HashSet<>();
   private List<OcTreeNodePlanarRegion> ocTreeNodePlanarRegions = new ArrayList<>();
   private final List<NormalOcTreeNode> nodesWithoutRegion = new ArrayList<>();
   private final Random random = new Random(234324L);

   public void compute(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      allRegionNodes.clear();

      ocTreeNodePlanarRegions.stream().forEach(region -> removeBadNodesFromRegion(boundingBox, parameters, region));
      ocTreeNodePlanarRegions = ocTreeNodePlanarRegions.stream().filter(region -> !region.isEmpty()).collect(Collectors.toList());
      ocTreeNodePlanarRegions.stream().forEach(region -> region.nodeStream().forEach(allRegionNodes::add));
      ocTreeNodePlanarRegions.stream().forEach(region -> growPlanarRegion(root, region, boundingBox, parameters));
      ocTreeNodePlanarRegions = ocTreeNodePlanarRegions.stream().filter(region -> region.getNumberOfNodes() > parameters.getMinRegionSize()).collect(Collectors.toList());
      ocTreeNodePlanarRegions.parallelStream().forEach(OcTreeNodePlanarRegion::recomputeNormalAndOrigin);
      ocTreeNodePlanarRegions.parallelStream().forEach(OcTreeNodePlanarRegionCalculator::flipNormalOfOutliers);
      
      Set<NormalOcTreeNode> nodeSet = new HashSet<>();
      nodeSet.clear();
      new OcTreeIterable<>(root, leafInBoundingBoxWithNormalSetRule(boundingBox)).forEach(nodeSet::add);
      nodeSet.removeAll(allRegionNodes);

      nodesWithoutRegion.clear();
      nodesWithoutRegion.addAll(nodeSet);

      ocTreeNodePlanarRegions.addAll(searchNewPlanarRegions(root, boundingBox, parameters, random));
      ocTreeNodePlanarRegions = mergePlanarRegionsIfPossible(root, ocTreeNodePlanarRegions, parameters);
   }

   public void removeDeadNodes()
   {
      ocTreeNodePlanarRegions.stream().forEach(region -> removeDeadNodesFromRegion(region));
   }

   public List<OcTreeNodePlanarRegion> getOcTreeNodePlanarRegions()
   {
      return ocTreeNodePlanarRegions;
   }

   public void clear()
   {
      ocTreeNodePlanarRegions.clear();
   }

   private IteratorSelectionRule<NormalOcTreeNode> leafInBoundingBoxWithNormalSetRule(OcTreeBoundingBoxInterface boundingBox)
   {
      IteratorSelectionRule<NormalOcTreeNode> isNormalSetRule = (node, maxDepth) -> node.isNormalSet();
      return OcTreeIteratorFactory.multipleRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(boundingBox), isNormalSetRule);
   }

   public static void flipNormalOfOutliers(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      Vector3d regionNormal = ocTreeNodePlanarRegion.getNormal();
      int numberOfNormalsFlipped = (int) ocTreeNodePlanarRegion.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).count();
      int numberOfNormalsNotFlipped = ocTreeNodePlanarRegion.getNumberOfNodes() - numberOfNormalsFlipped;

      // The majority of the nodes are flipped => flip the region normal
      if (numberOfNormalsFlipped > numberOfNormalsNotFlipped)
         regionNormal.negate();

      // Flip the nodes that upside down
      ocTreeNodePlanarRegion.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).forEach(NormalOcTreeNode::negateNormal);
   }

   private static boolean isNodeNormalFlipped(NormalOcTreeNode node, Vector3d referenceNormal)
   {
      return node.getNormalX() * referenceNormal.getX() + node.getNormalY() * referenceNormal.getY() + node.getNormalZ() * referenceNormal.getZ() < 0.0;
   }

   public static List<OcTreeNodePlanarRegion> mergePlanarRegionsIfPossible(NormalOcTreeNode root, List<OcTreeNodePlanarRegion> inputRegions, PlanarRegionSegmentationParameters parameters)
   {
      List<OcTreeNodePlanarRegion> mergedRegions = new ArrayList<>();
      while (!inputRegions.isEmpty())
      {
         OcTreeNodePlanarRegion candidateForMergeOtherRegions = inputRegions.get(0);
         Map<Boolean, List<OcTreeNodePlanarRegion>> mergeableAndNonMergeableGroups = inputRegions.subList(1, inputRegions.size()).parallelStream()
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

   public static boolean areRegionsMergeable(NormalOcTreeNode root, OcTreeNodePlanarRegion currentRegion, OcTreeNodePlanarRegion potentialRegionToMerge,
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
      double searchRadiusSquared = searchRadius * searchRadius;

      if (currentRegion.distanceSquaredFromOtherRegionBoundingBox(potentialRegionToMerge) > searchRadiusSquared)
         return false;

      OcTreeNodePlanarRegion regionToNavigate;
      OcTreeNodePlanarRegion otherRegion;

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
                             .filter(node -> otherRegion.distanceFromBoundingBox(node) < searchRadiusSquared)
                             .filter(node -> isNodeInOtherRegionNeighborhood(root, node, otherRegion, searchRadius))
                             .findFirst()
                             .isPresent();
   }

   public static boolean isNodeInOtherRegionNeighborhood(NormalOcTreeNode root, NormalOcTreeNode nodeFromOneRegion, OcTreeNodePlanarRegion otherRegion, double searchRadius)
   {
      MutableBoolean foundNeighborFromOtherRegion = new MutableBoolean(false);

      NeighborActionRule<NormalOcTreeNode> actionRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (otherRegion.contains(node))
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

   public List<OcTreeNodePlanarRegion> searchNewPlanarRegions(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters, Random random)
   {
      List<OcTreeNodePlanarRegion> newRegions = new ArrayList<>();

      float minNormalQuality = (float) parameters.getMinNormalQuality();

      for (NormalOcTreeNode node : nodesWithoutRegion)
      {
         if (node.getNormalAverageDeviation() > minNormalQuality)
            continue;

         int regionId = random.nextInt(Integer.MAX_VALUE);
         OcTreeNodePlanarRegion region = createNewOcTreeNodePlanarRegion(root, node, regionId, boundingBox, parameters);

         if (region.getNumberOfNodes() > parameters.getMinRegionSize())
            newRegions.add(region);
      }

      return newRegions;
   }

   public OcTreeNodePlanarRegion createNewOcTreeNodePlanarRegion(NormalOcTreeNode root, NormalOcTreeNode seedNode, int regionId, OcTreeBoundingBoxInterface boundingBox,
         PlanarRegionSegmentationParameters parameters)
   {
      OcTreeNodePlanarRegion newRegion = new OcTreeNodePlanarRegion(regionId);
      newRegion.addNode(seedNode);
      growPlanarRegion(root, newRegion, boundingBox, parameters);
      return newRegion;
   }

   public void growPlanarRegion(NormalOcTreeNode root, OcTreeNodePlanarRegion ocTreeNodePlanarRegion, OcTreeBoundingBoxInterface boundingBox,
         PlanarRegionSegmentationParameters parameters)
   {
      double searchRadius = parameters.getSearchRadius();
      
      Deque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
      Set<NormalOcTreeNode> newSetToExplore = new HashSet<>();

      NeighborActionRule<NormalOcTreeNode> extendSearchRule = neighborNode -> recordCandidatesForRegion(neighborNode, ocTreeNodePlanarRegion, newSetToExplore, boundingBox, parameters);
      ocTreeNodePlanarRegion.nodeParallelStream()
                  .filter(node -> isNodeInBoundingBox(node, boundingBox))
                  .forEach(regionNode -> OcTreeNearestNeighborTools.findRadiusNeighbors(root, regionNode, searchRadius, extendSearchRule));
      nodesToExplore.addAll(newSetToExplore);

      while (!nodesToExplore.isEmpty())
      {
         NormalOcTreeNode currentNode = nodesToExplore.poll();
         if (!ocTreeNodePlanarRegion.addNode(currentNode))
            continue;
         allRegionNodes.add(currentNode);
         newSetToExplore.clear();
         OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, extendSearchRule);
         nodesToExplore.addAll(newSetToExplore);
      }
   }

   public void recordCandidatesForRegion(NormalOcTreeNode neighborNode, OcTreeNodePlanarRegion ocTreeNodePlanarRegion, Set<NormalOcTreeNode> newSetToExplore, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      if (allRegionNodes.contains(neighborNode))
         return;
      if (!isNodeInBoundingBox(neighborNode, boundingBox))
         return;
      if (!isNodePartOfRegion(neighborNode, ocTreeNodePlanarRegion, parameters.getMaxDistanceFromPlane(), Math.cos(parameters.getMaxAngleFromPlane())))
         return;
      if (!neighborNode.isNormalSet() || !neighborNode.isHitLocationSet())
         return;
      
      newSetToExplore.add(neighborNode);
   }

   private static void removeBadNodesFromRegion(OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters,
         OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      List<NormalOcTreeNode> nodesToRemove = ocTreeNodePlanarRegion.nodeStream()
            .collect(Collectors.groupingBy(node -> isBadNode(node, ocTreeNodePlanarRegion, boundingBox, parameters)))
            .getOrDefault(true, Collections.emptyList());

      ocTreeNodePlanarRegion.removeNodesAndUpdate(nodesToRemove);
   }

   private static void removeDeadNodesFromRegion(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      List<NormalOcTreeNode> nodesToRemove = ocTreeNodePlanarRegion.nodeStream()
            .collect(Collectors.groupingBy(node -> isNodeDead(node)))
            .getOrDefault(true, Collections.emptyList());

      ocTreeNodePlanarRegion.removeNodesAndUpdate(nodesToRemove);
   }

   private static boolean isNodeInBoundingBox(NormalOcTreeNode node, OcTreeBoundingBoxInterface boundingBox)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(node.getX(), node.getY(), node.getZ());
   }

   private static boolean isBadNode(NormalOcTreeNode node, OcTreeNodePlanarRegion ocTreeNodePlanarRegion, OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      if (isNodeDead(node))
         return true;

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      return isNodeInBoundingBox(node, boundingBox) && !isNodePartOfRegion(node, ocTreeNodePlanarRegion, maxDistanceFromPlane, dotThreshold);
   }

   private static boolean isNodeDead(NormalOcTreeNode node)
   {
      return !node.isNormalSet();
   }

   public static boolean isNodePartOfRegion(NormalOcTreeNode node, OcTreeNodePlanarRegion ocTreeNodePlanarRegion, double maxDistanceFromPlane, double dotThreshold)
   {
      double absoluteOrthogonalDistance = ocTreeNodePlanarRegion.absoluteOrthogonalDistance(node);
      if (absoluteOrthogonalDistance > maxDistanceFromPlane)
         return false;

      double absoluteDot = ocTreeNodePlanarRegion.absoluteDotWithNodeNormal(node);
      return absoluteDot > dotThreshold;
   }
}
