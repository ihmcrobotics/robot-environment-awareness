package us.ihmc.robotEnvironmentAwareness.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

public class PointMean extends Point3d
{
   private static final long serialVersionUID = -3110940850302600107L;

   private int sampleSize = 0;

   public PointMean()
   {
   }

   public void update(Tuple3d tuple)
   {
      update(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public void update(Tuple3d tuple, int updateSize)
   {
      update(tuple.getX(), tuple.getY(), tuple.getZ(), updateSize);
   }

   public void update(double x, double y, double z)
   {
      update(x, y, z, 1);
   }

   public void update(double x, double y, double z, int updateSize)
   {
      sampleSize += updateSize;
      double nInv = (double) updateSize / (double) sampleSize;
      this.x += (x - this.x) * nInv;
      this.y += (y - this.y) * nInv;
      this.z += (z - this.z) * nInv;
   }

   public void clear()
   {
      sampleSize = 0;
      set(0.0, 0.0, 0.0);
   }

   public int getNumberOfSamples()
   {
      return sampleSize;
   }
}
