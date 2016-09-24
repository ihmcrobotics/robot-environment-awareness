package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.List;

public class ListTools
{
   /**
    * Safe increment that will always the next index that is inside [0, list.size() - 1].
    * @returns {@code (index + 1) % list.size()}.
    */
   public static int increment(int index, List<?> list)
   {
      return (index + 1) % list.size();
   }

   /**
    * Safe increment that will always the previous index that is inside [0, list.size() - 1].
    * @returns {@code (index - 1 + list.size()) % list.size()}.
    */
   public static int decrement(int index, List<?> list)
   {
      return (index - 1 + list.size()) % list.size();
   }

   /**
    * Calls {@link List#get(int)},
    * but first computes index %= list.size() such that the index is always inside [0, list.size() - 1].
    */
   public static <T> T getWrap(int index, List<T> list)
   {
      if (index == -1)
         return list.get(list.size() - 1);
      else
         return list.get(index % list.size());
   }

   /**
    * Returns the element that is right after the given index.
    */
   public static <T> T getNextWrap(int index, List<T> list)
   {
      return list.get(increment(index, list));
   }

   /**
    * Returns the element that is right before the given index.
    */
   public static <T> T getPreviousWrap(int index, List<T> list)
   {
      return list.get(decrement(index, list));
   }

   /**
    * Returns the number of elements between startIndex (included) and endIndex (included).
    */
   public static int subLengthInclusive(int startIndex, int endIndex, List<?> list)
   {
      if (endIndex == startIndex)
         return 1;
      else if (endIndex > startIndex)
         return endIndex - startIndex + 1;
      else
         return (endIndex + list.size()) - startIndex + 1;
   }

   /**
    * Returns the number of elements between startIndex (excluded) and endIndex (excluded).
    */
   public static int subLengthExclusive(int startIndex, int endIndex, List<?> list)
   {
      if (endIndex == startIndex)
         return 0;
      else if (endIndex > startIndex)
         return endIndex - startIndex - 1;
      else
         return (endIndex + list.size()) - startIndex - 1;
   }

   /**
    * Returns a list that contains all the elements of the input from startIndex (included) to endIndex (included).
    */
   public static <T> List<T> subListInclusive(int startIndex, int endIndex, List<T> input)
   {
      List<T> output = new ArrayList<>();
   
      int outputLenth = subLengthInclusive(startIndex, endIndex, input);
   
      int i = startIndex;
      while (output.size() != outputLenth)
         output.add(getWrap(i++, input));
   
      return output;
   }

   /**
    * Returns a list that contains all the elements of the input from startIndex (excluded) to endIndex (excluded).
    */
   public static <T> List<T> subListExclusive(int startIndex, int endIndex, List<T> input)
   {
      List<T> output = new ArrayList<>();

      if (startIndex == endIndex)
         return output;
   
      int outputLenth = subLengthExclusive(startIndex, endIndex, input);
   
      int i = startIndex;
      while (output.size() != outputLenth)
         output.add(getWrap(i++, input));
   
      return output;
   }

   /**
    * Removes all the elements from the list from startIndex (included) to endIndex (included).
    */
   public static int removeAllInclusive(int startIndex, int endIndex, List<?> list)
   {
      int numberOfElementsToRemove = subLengthInclusive(startIndex, endIndex, list);
   
      for (int count = 0; count < numberOfElementsToRemove; count++)
         list.remove(increment(startIndex, list));

      return numberOfElementsToRemove;
   }

   /**
    * Removes all the elements from the list from startIndex (excluded) to endIndex (excluded).
    */
   public static int removeAllExclusive(int startIndex, int endIndex, List<?> list)
   {
      int numberOfElementsToRemove = subLengthExclusive(startIndex, endIndex, list);
   
      for (int count = 0; count < numberOfElementsToRemove; count++)
         list.remove(increment(startIndex, list));

      return numberOfElementsToRemove;
   }
}
