package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.geometry.GeometryTools.cross;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

/**
 * This class gathers different filters on a concave hull.
 * The filters aim to reduce the hull complexity by removing less significant vertices.
 * The filters gathered here only tend to reduce the area of the concave hull.
 * 
 * @author Sylvain
 *
 */
public class ConcaveHullPruningFilteringTools
{
   /**
    * Filter out vertices that create "peaks".
    * Peaks are identified by a threshold on the angle between two consecutive edges.
    * Only convex peaks or shallow angles are removed, meaning this filter only reduces the area of the concave hull.
    * @param peakAngleThreshold
    * @param concaveHullVerticesToFilter
    * @return the number of vertices removed
    */
   public static int filterOutPeaksAndShallowAngles(double concaveAngleLimit, double shallowAngleThreshold, double peakAngleThreshold,
         List<Point2d> concaveHullVerticesToFilter)
   {
      int nVerticesRemoved = 0;
      double concaveLimit = Math.sin(concaveAngleLimit);
      double cosPeakAngle = Math.cos(peakAngleThreshold);
      double cosShallowAngle = Math.cos(shallowAngleThreshold);

      // abc represent a triangle formed by three successive vertices.
      // At each step of the iteration, b is tested to see if it can be removed.
      Point2d a = concaveHullVerticesToFilter.get(0);
      Point2d b = concaveHullVerticesToFilter.get(1);
      Point2d c = concaveHullVerticesToFilter.get(2);

      Vector2d ab = new Vector2d();
      Vector2d ac = new Vector2d();
      Vector2d bc = new Vector2d();
      ab.sub(b, a);
      ac.sub(c, a);
      bc.sub(c, b);
      ab.normalize();
      ac.normalize();
      bc.normalize();

      for (int currentIndex = 0; currentIndex < concaveHullVerticesToFilter.size();)
      {
         // The cross-product is used to detect if b is on the outside of the line ac, two cases from there:
         // - b is outside, removing it means reducing the concave hull area => good
         // - b is inside, removing it means augmenting the concave hull area => not good here
         double cross = cross(ab, ac);
         // The dot product is used to evaluate the angle at b. If it is too small or too large b may be removed
         double dot = ab.dot(bc);

         if (cross < concaveLimit && (dot < cosPeakAngle || dot > cosShallowAngle))
         {
            concaveHullVerticesToFilter.remove((currentIndex + 1) % concaveHullVerticesToFilter.size());
            b = c;
            ab.set(ac);
            nVerticesRemoved++;
         }
         else
         {
            a = b;
            b = c;
            ab.set(bc);
            currentIndex++;
         }

         c = concaveHullVerticesToFilter.get((currentIndex + 2) % concaveHullVerticesToFilter.size());
         ac.sub(c, a);
         ac.normalize();
         bc.sub(c, b);
         bc.normalize();
      }
      return nVerticesRemoved;
   }
}
