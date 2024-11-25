#ifndef SEQUENTIAL_AUCTION_H
#define SEQUENTIAL_AUCTION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <set>
#include <vector>
#include "task_msgs/Task.h"
#include "task_msgs/TaskArray.h"
#include "geometry_msgs/Pose.h"

using geometry_msgs::Pose;
using std::cout;
using std::endl;
using std::set;
using std::vector;
using task_msgs::Task;
using task_msgs::TaskArray;

class SequentialAuction
{
public:
  bool use_least_contested_bid;
  bool return_home;

  SequentialAuction(vector<Task> unallocated_tasks, vector<Pose> robot_poses, vector<set<int>> feasible_tasks = {});
  vector<TaskArray> allocateTasks();
  vector<vector<int>> getPaths() const
  {
    return paths;
  }

private:
  // These variables are unchanged through allocation.
  int num_robots;
  int num_tasks;
  vector<Pose> robot_poses;
  vector<Task> tasks;
  vector<set<int>> feasible_tasks;
  // Working variables for alloction.
  vector<int> unalloc;
  vector<vector<int>> paths;  // consider changing to lists for efficiency
  vector<double> path_costs;
  vector<vector<double>> bids;

  // Output variable from allocation.
  vector<TaskArray> allocations;

  void formOutput();
  void processWinner(int winning_robot, int winning_task);
  void selectWinner(int& winning_robot, int& winning_task);
  void selectLeastContestedWinner(int& winning_robot, int& winning_task);
  void prepareAllocations();
  void calculateAllBids();
  void calculateBids(int robot_num);
  double insertTask(int robot_num, int unalloc_id, vector<int>& new_path);
  double calculatePathCost(int robot_num, vector<int> path);
  void printPaths();
  void printPath(vector<int> path);
  void printBids();
};

#endif  // SEQUENTIAL_AUCTION_H
