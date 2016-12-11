package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import javax.vecmath.Point2d;

public class ConcaveHullCollection implements Iterable<ConcaveHull>
{
   private final Set<ConcaveHull> concaveHulls = new HashSet<>();

   public ConcaveHullCollection()
   {
   }

   public ConcaveHullCollection(ConcaveHull concaveHull)
   {
      add(concaveHull);
   }

   public ConcaveHullCollection(List<Point2d> concaveHullVertices)
   {
      add(concaveHullVertices);
   }

   public boolean add(List<Point2d> newConcaveHullVertices)
   {
      return add(new ConcaveHull(newConcaveHullVertices));
   }

   public boolean add(ConcaveHull newConcaveHull)
   {
      return concaveHulls.add(newConcaveHull);
   }

   public boolean remove(ConcaveHull newConcaveHull)
   {
      return concaveHulls.remove(newConcaveHull);
   }

   public int getNumberOfConcaveHulls()
   {
      return concaveHulls.size();
   }

   public boolean isEmpty()
   {
      return concaveHulls.isEmpty();
   }

   public Collection<ConcaveHull> getConcaveHulls()
   {
      return concaveHulls;
   }

   @Override
   public Iterator<ConcaveHull> iterator()
   {
      return concaveHulls.iterator();
   }

   @Override
   public int hashCode()
   {
      int hashCode = 1;
      for (ConcaveHull hull : this)
         hashCode = 31 * hashCode + (hull == null ? 0 : hull.hashCode());
      return hashCode;
   }
}
