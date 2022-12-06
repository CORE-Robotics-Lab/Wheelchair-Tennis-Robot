#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <mutex>
#include <thread>

#include <wtr_navigation/TebPlans.h>

using namespace teb_local_planner;

std::vector<HomotopyClassPlannerPtr> planners;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebVisualizationPtr visual;
TebConfig config;  // May need to create a multiple of these if planners' ref complains
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;

int thread_count;
bool visualize_best;

// Just copied function to split vector into equal parts
// https://stackoverflow.com/questions/6861089/how-to-split-a-vector-into-n-almost-equal-parts
template<typename T>
std::vector<std::vector<T>> SplitVector(const std::vector<T>& vec, size_t n) {
    std::vector<std::vector<T>> outVec;

    size_t length = vec.size() / n;
    size_t remain = vec.size() % n;

    size_t begin = 0;
    size_t end = 0;

    for (size_t i = 0; i < std::min(n, vec.size()); ++i) {
        end += (remain > 0) ? (length + !!(remain--)) : length;
        outVec.push_back(std::vector<T>(vec.begin() + begin, vec.begin() + end));
        begin = end;
    }
    return outVec;
}

bool CB_makePlan(wtr_navigation::TebPlans::Request& request, wtr_navigation::TebPlans::Response& response)
{
    ros::Time begin = ros::Time::now();

    // Split Goals so multiple threads can work independently
    auto split_goals = SplitVector<geometry_msgs::PoseStamped>(request.goals, thread_count);

    // Also, store indices in order know which goal was best
    std::vector<int> indices(request.goals.size());
    std::iota(indices.begin(), indices.end(), 0); // vector idx = range(0, n)
    auto split_idxs = SplitVector<int>(indices, thread_count);

    std::mutex trajectories_mutex;
    std::vector<teb_local_planner::TrajectoryMsg> global_trajectories;
    std::vector<int> global_indices;
    std::vector<double> global_wait_times;
    PoseSE2 start(request.start);
    ros::Time now = ros::Time::now();

    // Construct Thread Worker
    auto worker = [&] (int thread_idx) {
      std::vector<teb_local_planner::TrajectoryMsg> local_trajectories;
      std::vector<int> local_indices;
      std::vector<double> local_wait_times;

      // Iterate over goals, plan, and store trajectory
      for (int i = 0; i < split_goals[thread_idx].size(); i++) {
        auto goal = split_goals[thread_idx][i];
        planners[thread_idx]->plan(start, PoseSE2(goal.pose));

        teb_local_planner::TrajectoryMsg trajectory_msg;
        planners[thread_idx]->bestTeb()->getFullTrajectory(trajectory_msg.trajectory);
        if (now + trajectory_msg.trajectory.back().time_from_start < goal.header.stamp) { // Only add if can execute in time
          local_trajectories.push_back(trajectory_msg);
          local_indices.push_back(split_idxs[thread_idx][i]);
          local_wait_times.push_back((goal.header.stamp - (now + trajectory_msg.trajectory.back().time_from_start)).toSec());
        }
      }

      // Save local trajectories (thread-safe)
      std::lock_guard lock(trajectories_mutex); // Unlocks automatically when fall out of scope
      global_trajectories.insert(global_trajectories.end(), local_trajectories.begin(), local_trajectories.end());
      global_indices.insert(global_indices.end(), local_indices.begin(), local_indices.end());
      global_wait_times.insert(global_wait_times.end(), local_wait_times.begin(), local_wait_times.end());
    };

    // Start Worker Threads
    std::vector<std::thread> threads;
    int num_threads = std::min((size_t) thread_count, split_goals.size());
    for (int th_idx = 0; th_idx < num_threads; th_idx++) {
      threads.emplace_back(worker, th_idx);
    }

    for (std::thread& t : threads) {
      t.join();
    }

    // Find Min Element
    auto max_wait_ptr = std::max_element(global_wait_times.begin(), global_wait_times.end());

    ROS_INFO_STREAM("Time Exec: " << (ros::Time::now() - begin).toSec());

    // Didn't find any goals that are reachable in time
    if (max_wait_ptr == global_wait_times.end()) {
      response.success = false;
      return true;
    }

    auto best_traj = global_trajectories[max_wait_ptr - global_wait_times.begin()];
    int best_idx = global_indices[max_wait_ptr - global_wait_times.begin()];

    // Visaulize Path
    if (visualize_best) {
      std::vector<geometry_msgs::PoseStamped> posesStamped;
      for (const auto& pnt : best_traj.trajectory) {
          geometry_msgs::PoseStamped poseStamped;
          poseStamped.header.frame_id = "world";
          poseStamped.pose = pnt.pose;
          posesStamped.push_back(poseStamped);
      }
      visual->publishLocalPlan(posesStamped);
    }

    // Make Response
    response.best_trajectory = best_traj;
    response.best_index = best_idx;
    response.success = true;

    return true;
}


void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "teb_server");
    ros::NodeHandle nh("~");

    // load ros parameters from node handle
    config.loadRosParamFromNodeHandle(nh);

    // setup dynamic reconfigure
    dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
    dynamic_recfg->setCallback(cb);

    // Setup robot shape model
    RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(nh);

    // Setup visualization
    visual = TebVisualizationPtr(new TebVisualization(nh, config));

    nh.param("thread_count", thread_count, 20);
    nh.param("visualize_best", visualize_best, true);

    // Setup planner (homotopy class planning or just the local teb planner)
    for (int i = 0; i < thread_count; i++) {
      planners.push_back(HomotopyClassPlannerPtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points)));
    }

    // Server
    ros::ServiceServer service = nh.advertiseService("make_plan", CB_makePlan);

    ros::spin();

    return 0;
}