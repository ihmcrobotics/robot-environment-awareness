package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.robotics.random.RandomTools;

public class ListToolsTest
{
   @Test
   public void testRemoveAllExclusive()
   {
      Random random = new Random(453L);
      List<Point2d> list = createRandomList(9, random);
      int from = 8;
      int to = 1;
      int n = ListTools.removeAllExclusive(from, to, list);
      assertEquals(1, n); 
      assertEquals(8, list.size());

      list = createRandomList(12, random);
      from = 11;
      to = 3;
      n = ListTools.removeAllExclusive(from, to, list);
      assertEquals(3, n);
      assertEquals(9, list.size());

      list = createRandomList(20, random);
      from = 11;
      to = 18;
      n = ListTools.removeAllExclusive(from, to, list);
      assertEquals(6, n);
      assertEquals(14, list.size());
   }

   private static List<Point2d> createRandomList(int size, Random random)
   {
      List<Point2d> ret = new ArrayList<>();
      while (ret.size() < size)
         ret.add(RandomTools.generateRandomPoint2d(random, 1.0, 1.0));
      return ret;
   }
}
