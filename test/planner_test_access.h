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
            static bool colliding(Planner & planner) { return planner.colliding_; }
            static void setManualMode(Planner & planner, bool value) { planner.manualSelectionMode_ = value; }
            static bool manualMode(Planner & planner) { return planner.manualSelectionMode_; }
            static void setHaveTFs(Planner & planner, bool value) { planner.haveTFs_ = value; }
            static void setRobotFrame(Planner & planner, const std::string & frame) { planner.cfg_.robot_frame_id = frame; }
            static void setRobotInscribedRadius(Planner & planner, float r) { planner.cfg_.rbt.r_inscr = r; }
            static void setGapDetector(Planner & planner, GapDetector * detector) { planner.gapDetector_ = detector; }
            static void setAgentVisualizer(Planner & planner, AgentVisualizer * visualizer) { planner.agentVisualizer_ = visualizer; }
            static GapDetector * makeGapDetector(Planner & planner) { return new GapDetector(planner.cfg_); }
            static AgentVisualizer * makeAgentVisualizer(Planner & planner) { return new AgentVisualizer(planner.nh_, planner.cfg_); }
            static tf2_ros::Buffer & tfBuffer(Planner & planner) { return planner.tfBuffer_; }

            static boost::shared_ptr<sensor_msgs::LaserScan const> scan(Planner & planner) { return planner.scan_; }
            static void laserScanCB(Planner & planner, boost::shared_ptr<sensor_msgs::LaserScan> scan) { planner.laserScanCB(scan); }

            static void pedOdomCB(Planner & planner, const pedsim_msgs::AgentStatesConstPtr & msg) { planner.pedOdomCB(msg); }
            static void scenarioResetCB(Planner & planner, const std_msgs::Int16::ConstPtr & msg) { planner.scenarioResetCB(msg); }
            static const std::map<std::string, geometry_msgs::Pose> & agentPoses(Planner & planner) { return planner.currentTrueAgentPoses_; }
            static const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels(Planner & planner) { return planner.currentTrueAgentVels_; }

            static void trajCreationPointCB(Planner & planner, const geometry_msgs::PointStamped::ConstPtr & msg) { planner.trajCreationPointCB(msg); }
            static bool hasTrajCreationPoint(Planner & planner) { return planner.hasTrajCreationPoint_; }
            static void setTrackingCreatedTraj(Planner & planner, bool value) { planner.isTrackingCreatedTraj_ = value; }
            static bool trackingCreatedTraj(Planner & planner) { return planner.isTrackingCreatedTraj_; }
    };
}
