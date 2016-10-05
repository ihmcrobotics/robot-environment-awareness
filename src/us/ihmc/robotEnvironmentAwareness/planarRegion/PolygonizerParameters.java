package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Scanner;

import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;

public class PolygonizerParameters
{
   /**
    * Threshold used when creating a new {@link ConcaveHull}.
    * <p>
    * Uses the Duckham and al. (2008) algorithm defined in the paper
    * untitled "Efficient generation of simple polygons for characterizing
    * the shape of a set of points in the plane".
    */
   private double concaveHullThreshold;
   /** The minimum number of nodes required for a {@link PlanarRegion} to be polygonized. */
   private int minNumberOfNodes;
   /** Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle. */
   private double shallowAngleThreshold;
   /** Filter parameter on the concave hull of a region. Used to removed vertices that create peaks. */
   private double peakAngleThreshold;
   /** Filter parameter on the concave hull of a region. Used to removed short edges. */
   private double lengthThreshold;
   /**
    * Threshold used for decomposing the concave hull into convex polygons.
    * Describes the maximum depth of a concavity before the concave hull gets split in 2.
    */
   private double depthThreshold;

   public PolygonizerParameters()
   {
      setDefaultParameters();
   }

   public PolygonizerParameters(PolygonizerParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      concaveHullThreshold = 0.2;
      minNumberOfNodes = 10;
      shallowAngleThreshold = Math.toRadians(1.0);
      peakAngleThreshold = Math.toRadians(120.0);
      lengthThreshold = 0.05;
      depthThreshold = 0.10;
   }

   public void set(PolygonizerParameters other)
   {
      concaveHullThreshold = other.concaveHullThreshold;
      minNumberOfNodes = other.minNumberOfNodes;
      shallowAngleThreshold = other.shallowAngleThreshold;
      peakAngleThreshold = other.peakAngleThreshold;
      lengthThreshold = other.lengthThreshold;
      depthThreshold = other.depthThreshold;
   }

   public void setConcaveHullThreshold(double concaveHullThreshold)
   {
      this.concaveHullThreshold = concaveHullThreshold;
   }

   public void setMinNumberOfNodes(int minNumberOfNodes)
   {
      this.minNumberOfNodes = minNumberOfNodes;
   }

   public void setShallowAngleThreshold(double shallowAngleThreshold)
   {
      this.shallowAngleThreshold = shallowAngleThreshold;
   }

   public void setPeakAngleThreshold(double peakAngleThreshold)
   {
      this.peakAngleThreshold = peakAngleThreshold;
   }

   public void setLengthThreshold(double lengthThreshold)
   {
      this.lengthThreshold = lengthThreshold;
   }

   public void setDepthThreshold(double depthThreshold)
   {
      this.depthThreshold = depthThreshold;
   }

   public double getConcaveHullThreshold()
   {
      return concaveHullThreshold;
   }

   public int getMinNumberOfNodes()
   {
      return minNumberOfNodes;
   }

   public double getShallowAngleThreshold()
   {
      return shallowAngleThreshold;
   }

   public double getPeakAngleThreshold()
   {
      return peakAngleThreshold;
   }

   public double getLengthThreshold()
   {
      return lengthThreshold;
   }

   public double getDepthThreshold()
   {
      return depthThreshold;
   }

   @Override
   public String toString()
   {
      return "concaveHullThreshold: " + concaveHullThreshold + ", minNumberOfNodes: " + minNumberOfNodes + ", shallowAngleThreshold: " + shallowAngleThreshold
            + ", peakAngleThreshold: " + peakAngleThreshold + ", lengthThreshold: " + lengthThreshold + ", depthThreshold: " + depthThreshold;
   }

   public static PolygonizerParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      while (!scanner.hasNextDouble())
         scanner.next();
      double concaveHullThreshold = scanner.nextDouble();
      while (!scanner.hasNextInt())
         scanner.next();
      int minNumberOfNodes = scanner.nextInt();
      while (!scanner.hasNextDouble())
         scanner.next();
      double shallowAngleThreshold = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double peakAngleThreshold = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double lengthThreshold = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double depthThreshold = scanner.nextDouble();
      scanner.close();

      PolygonizerParameters parameters = new PolygonizerParameters();
      parameters.setConcaveHullThreshold(concaveHullThreshold);
      parameters.setMinNumberOfNodes(minNumberOfNodes);
      parameters.setShallowAngleThreshold(shallowAngleThreshold);
      parameters.setPeakAngleThreshold(peakAngleThreshold);
      parameters.setLengthThreshold(lengthThreshold);
      parameters.setDepthThreshold(depthThreshold);
      return parameters;
   }
}