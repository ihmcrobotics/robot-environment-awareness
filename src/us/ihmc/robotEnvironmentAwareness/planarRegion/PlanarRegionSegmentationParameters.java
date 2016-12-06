package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Scanner;

public class PlanarRegionSegmentationParameters
{
   public static final double DEFAULT_SEARCH_RADIUS = 0.05;
   public static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.05;
   public static final double DEFAULT_MAX_ANGLE_FROM_PLANE = Math.toRadians(10.0);
   public static final double DEFAULT_MIN_NORMAL_QUALITY = 0.005;
   public static final int DEFAULT_MIN_REGION_SIZE = 50;

   private double searchRadius;
   private double maxDistanceFromPlane;
   private double maxAngleFromPlane;
   private double minNormalQuality;
   private int minRegionSize;

   public PlanarRegionSegmentationParameters()
   {
      setDefaultParameters();
   }

   public PlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      searchRadius = DEFAULT_SEARCH_RADIUS;
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
      maxAngleFromPlane = DEFAULT_MAX_ANGLE_FROM_PLANE;
      minNormalQuality = DEFAULT_MIN_NORMAL_QUALITY;
      minRegionSize = DEFAULT_MIN_REGION_SIZE;
   }

   public void set(PlanarRegionSegmentationParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      maxAngleFromPlane = other.maxAngleFromPlane;
      minNormalQuality = other.minNormalQuality;
      minRegionSize = other.minRegionSize;
   }

   public void setSearchRadius(double searchRadius)
   {
      this.searchRadius = searchRadius;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public void setMaxAngleFromPlane(double maxAngleFromPlane)
   {
      this.maxAngleFromPlane = maxAngleFromPlane;
   }

   public void setMinNormalQuality(double minNormalQuality)
   {
      this.minNormalQuality = minNormalQuality;
   }

   public void setMinRegionSize(int minRegionSize)
   {
      this.minRegionSize = minRegionSize;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }

   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   public double getMaxAngleFromPlane()
   {
      return maxAngleFromPlane;
   }

   public double getMinNormalQuality()
   {
      return minNormalQuality;
   }

   public int getMinRegionSize()
   {
      return minRegionSize;
   }
   
   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane + ", maxAngleFromPlane: " + maxAngleFromPlane + ", minNormalQuality: " + minNormalQuality + ", min region size: " + minRegionSize;
   }

   public static PlanarRegionSegmentationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      while (!scanner.hasNextDouble())
         scanner.next();
      double searchRadius = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxDistanceFromPlane = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxAngleFromPlane = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double minNormalQuality = scanner.nextDouble();
      while (!scanner.hasNextInt())
         scanner.next();
      int minRegionSize = scanner.nextInt();
      scanner.close();
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();
      parameters.setSearchRadius(searchRadius);
      parameters.setMaxDistanceFromPlane(maxDistanceFromPlane);
      parameters.setMaxAngleFromPlane(maxAngleFromPlane);
      parameters.setMinNormalQuality(minNormalQuality);
      parameters.setMinRegionSize(minRegionSize);
      return parameters;
   }
}
