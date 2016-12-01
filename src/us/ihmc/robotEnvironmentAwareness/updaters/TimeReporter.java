package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import org.apache.commons.lang3.time.StopWatch;

import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;

public class TimeReporter
{
   private final AtomicLong minimumNanoTimeToReport = new AtomicLong(TimeTools.milliSecondsToNanoSeconds(100L));
   private final AtomicBoolean reportTimeEnabled = new AtomicBoolean(false);
   private final ThreadLocal<StopWatch> stopWatchLocal = ThreadLocal.withInitial(() -> new StopWatch());
   private final Object caller;

   public TimeReporter(Object caller)
   {
      this.caller = caller;
   }

   public void enableTimeReport(boolean value)
   {
      reportTimeEnabled.set(value);
   }

   public void mininmumTimeToReport(long minTimeInMilliseconds)
   {
      minimumNanoTimeToReport.set(TimeTools.milliSecondsToNanoSeconds(minTimeInMilliseconds));
   }

   public void run(Runnable command, String timeReportPrefix)
   {
      if (reportTimeEnabled.get())
      {
         StopWatch stopWatch = stopWatchLocal.get();
         stopWatch.reset();
         stopWatch.start();
         command.run();
         PrintTools.info(caller, timeReportPrefix + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()));
      }
      else
         command.run();
   }
}
