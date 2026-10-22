#include <gtest/gtest.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int16.h>

#include "planner_test_access.h"

using dynamic_gap::Planner;
using dynamic_gap::PlannerTestAccess;
using dynamic_gap::TimeKeeper;
using dynamic_gap::Trajectory;

// Fix A: one candidate list; display ids are dense over shown candidates
TEST(ManualSelectionHelpers, DenseDisplayIds)
{
    EXPECT_EQ((std::vector<int>{0, -1, 1, 2, -1}), dynamic_gap::denseDisplayIds({true, false, true, true, false}));
    EXPECT_TRUE(dynamic_gap::denseDisplayIds({}).empty());
}

TEST(ManualSelectionHelpers, NearestFeasibleCandidate)
{
    std::vector<std::pair<float, float>> terminals = {{0.0f, 0.0f}, {1.0f, 0.0f}, {5.0f, 0.0f}};
    // the infeasible candidate at (1, 0) is skipped
    EXPECT_EQ(0, dynamic_gap::nearestCandidate(terminals, {true, false, true}, 1.1f, 0.0f));
    EXPECT_EQ(1, dynamic_gap::nearestCandidate(terminals, {true, true, true}, 1.1f, 0.0f));
    EXPECT_EQ(2, dynamic_gap::nearestCandidate(terminals, {true, true, true}, 4.0f, 0.0f));
    EXPECT_EQ(-1, dynamic_gap::nearestCandidate(terminals, {false, false, false}, 1.0f, 0.0f));
    // ties go to the lower index
    EXPECT_EQ(0, dynamic_gap::nearestCandidate({{1.0f, 0.0f}, {-1.0f, 0.0f}}, {true, true}, 0.0f, 0.0f));
}

TEST(ManualSelectionHelpers, CursorFreshnessIgnoresMouseMovement)
{
    // the tool republishes at 10 Hz whether or not the mouse moves; only the
    // receipt age and the active flag count
    EXPECT_TRUE(dynamic_gap::isCursorFresh(true, 0.1, 0.5));
    EXPECT_FALSE(dynamic_gap::isCursorFresh(true, 0.6, 0.5));
    EXPECT_FALSE(dynamic_gap::isCursorFresh(false, 0.1, 0.5));
}

TEST(ManualSelectionHelpers, ResetWindow)
{
    EXPECT_TRUE(dynamic_gap::isInResetWindow(3, 2, 10.0, 1.0));   // reset 3 in progress
    EXPECT_TRUE(dynamic_gap::isInResetWindow(3, 3, 0.5, 1.0));    // just after reset 3
    EXPECT_FALSE(dynamic_gap::isInResetWindow(3, 3, 1.5, 1.0));
    EXPECT_FALSE(dynamic_gap::isInResetWindow(-1, -1, -1.0, 1.0)); // no reset seen
}

TEST(ManualSelectionHelpers, DownsamplePathKeepsLastPose)
{
    geometry_msgs::PoseArray path;
    for (int i = 0; i < 12; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = i;
        path.poses.push_back(pose);
    }
    std::vector<geometry_msgs::Point> points = dynamic_gap::downsamplePath(path, 5);
    ASSERT_EQ(4u, points.size());
    EXPECT_EQ(0.0, points[0].x);
    EXPECT_EQ(5.0, points[1].x);
    EXPECT_EQ(10.0, points[2].x);
    EXPECT_EQ(11.0, points[3].x);
    EXPECT_TRUE(dynamic_gap::downsamplePath(geometry_msgs::PoseArray(), 5).empty());
}

TEST(ManualSelectionHelpers, ScopeExitRunsOnEveryReturn)
{
    int calls = 0;
    auto run = [&](bool early)
    {
        dynamic_gap::ScopeExit<std::function<void()>> guard([&]() { calls++; });
        if (early)
            return;
    };
    run(true);
    run(false);
    EXPECT_EQ(2, calls);
}

namespace
{
    class ManualPlannerTest : public ::testing::Test
    {
        protected:
            void SetUp() override
            {
                PlannerTestAccess::setTimeKeeper(planner_, new TimeKeeper());
                PlannerTestAccess::setManualMode(planner_, true);
                PlannerTestAccess::setOdomFrame(planner_, "odom");
                PlannerTestAccess::setHaveTFs(planner_, true);

                // map -> odom: odom origin at (1, 2) in the map
                geometry_msgs::TransformStamped mapToOdom;
                mapToOdom.header.frame_id = "odom";
                mapToOdom.child_frame_id = "map";
                mapToOdom.transform.translation.x = -1.0;
                mapToOdom.transform.translation.y = -2.0;
                mapToOdom.transform.rotation.w = 1.0;
                PlannerTestAccess::tfBuffer(planner_).setTransform(mapToOdom, "test", true);
            }

            manual_gap_msgs::CursorState::Ptr cursor(bool active, bool pickOk, double x, double y)
            {
                manual_gap_msgs::CursorState::Ptr msg(new manual_gap_msgs::CursorState());
                msg->header.frame_id = "map";
                msg->header.stamp = ros::Time::now();
                msg->tool_active = active;
                msg->pick_ok = pickOk;
                msg->point.x = x;
                msg->point.y = y;
                return msg;
            }

            Planner planner_;
    };
}

// Cursor: own frame transformed into odom; a failed pick keeps the point;
// an inactive tool removes it
TEST_F(ManualPlannerTest, CursorTransformedFromItsFrame)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    ASSERT_TRUE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).x);
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).y);
    EXPECT_TRUE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, FailedPickKeepsLastPoint)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    PlannerTestAccess::cursorCB(planner_, cursor(true, false, 9.0, 9.0));
    EXPECT_TRUE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).x);
    EXPECT_TRUE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, InactiveToolRemovesCursor)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    PlannerTestAccess::cursorCB(planner_, cursor(false, false, 0.0, 0.0));
    EXPECT_FALSE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_FALSE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, UnknownFrameIsIgnored)
{
    manual_gap_msgs::CursorState::Ptr msg = cursor(true, true, 3.0, 4.0);
    msg->header.frame_id = "nowhere";
    PlannerTestAccess::cursorCB(planner_, msg);
    EXPECT_FALSE(PlannerTestAccess::hasCursorPos(planner_));
}

// PL-16: no gaps in manual mode -> hold still; the controller returns zero
TEST_F(ManualPlannerTest, NoGapsHoldsStill)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);

    Trajectory chosenTraj;
    int trajFlag = dynamic_gap::GAP;
    planner_.runPlanningLoop(chosenTraj, trajFlag);

    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(PlannerTestAccess::holdStill(planner_));
}

TEST_F(ManualPlannerTest, HoldStillGivesZeroVelocity)
{
    PlannerTestAccess::setHoldStill(planner_, true);
    int trajFlag = dynamic_gap::NONE;
    geometry_msgs::Twist cmd = planner_.ctrlGeneration(geometry_msgs::PoseArray(), trajFlag);
    EXPECT_EQ(0.0, cmd.linear.x);
    EXPECT_EQ(0.0, cmd.linear.y);
    EXPECT_EQ(0.0, cmd.angular.z);
}

// Reset: gaps of the old scene and click-to-move state are dropped (manual mode)
TEST_F(ManualPlannerTest, ResetClearsManualState)
{
    PlannerTestAccess::addGaps(planner_, 2);
    PlannerTestAccess::setHasTrajCreationPoint(planner_, true);
    PlannerTestAccess::setTrackingCreatedTraj(planner_, true);

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 4;
    PlannerTestAccess::scenarioResetCB(planner_, reset);

    EXPECT_EQ(0u, PlannerTestAccess::gapCount(planner_));
    EXPECT_FALSE(PlannerTestAccess::hasTrajCreationPoint(planner_));
    EXPECT_FALSE(PlannerTestAccess::trackingCreatedTraj(planner_));
    EXPECT_EQ(4, PlannerTestAccess::episodeId(planner_));
}

TEST_F(ManualPlannerTest, ResetKeepsGapsInAutonomousMode)
{
    PlannerTestAccess::setManualMode(planner_, false);
    PlannerTestAccess::addGaps(planner_, 2);

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 1;
    PlannerTestAccess::scenarioResetCB(planner_, reset);

    EXPECT_EQ(8u, PlannerTestAccess::gapCount(planner_));
}
