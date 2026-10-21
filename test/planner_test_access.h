#pragma once

#include <dynamic_gap/Planner.h>

namespace dynamic_gap
{
    /**
    * \brief Test helper with access to Planner's internal state (declared friend in Planner.h).
    */
    class PlannerTestAccess
    {
        public:
            static void setTimeKeeper(Planner & planner, TimeKeeper * timeKeeper) { planner.timeKeeper_ = timeKeeper; }
            static void setInitialized(Planner & planner, bool value) { planner.initialized_ = value; }
            static void setHasLaserScan(Planner & planner, bool value) { planner.hasLaserScan_ = value; }
            static void setHasGlobalGoal(Planner & planner, bool value) { planner.hasGlobalGoal_ = value; }
            static void setColliding(Planner & planner, bool value) { planner.colliding_ = value; }
    };
}
