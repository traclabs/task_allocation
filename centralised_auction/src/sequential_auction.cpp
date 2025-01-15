#include <cmath>
#include <sstream>
#include "centralised_auction/sequential_auction.h"

static bool isTaskLessContentious(double min_bid, double bid_diff, double winning_bid, double winning_bid_diff);

/********************************************
 * Public functions (high level).
 ********************************************/

// Constructor.
SequentialAuction::SequentialAuction(vector<Task> unallocated_tasks, vector<Pose> robot_poses,
                                     vector<set<int>> feasible_tasks)
{
  this->tasks = unallocated_tasks;
  this->num_tasks = unallocated_tasks.size();
  this->robot_poses = robot_poses;
  this->num_robots = robot_poses.size();
  this->feasible_tasks = feasible_tasks;
  if (feasible_tasks.empty())
  {
    ROS_INFO("Feasible task set not provided. Assuming all tasks are feasible for all robots.");
  }
  else
  {
    ROS_ASSERT_MSG(robot_poses.size() == feasible_tasks.size(),
                   "%zu feasible task sets were provided. Expected %zu, one for each robot.", feasible_tasks.size(),
                   robot_poses.size());
  }
  return_home = false;
  use_least_contested_bid = true;
}

// The allocation procedure. Each iteration, selects a winner to allocate and
// recalculates bids.
vector<TaskArray> SequentialAuction::allocateTasks()
{
  int winning_robot, winning_task;
  prepareAllocations();
  calculateAllBids();
  cout << "The starting bids are: " << endl;
  cout << bid_mat_ << std::endl;
  while (!unalloc.empty())
  {

    Eigen::Index row, col;
    bid_mat_.maxCoeff(&row, &col);
    cout << "The bids are: " << endl;
    cout << bid_mat_ << std::endl;
    Eigen::VectorXd tmp_col = bid_mat_.col(col);
    std::vector<double> colv{tmp_col.data(), tmp_col.data() + tmp_col.size()};
    std::sort(colv.begin(),colv.end());
    int last_task = std::max(num_tasks-2, 0);
    Eigen::VectorXd tmp_row = bid_mat_.row(row);
    std::vector<double> rowv{tmp_row.data(), tmp_row.data() + tmp_row.size()};
    std::sort(rowv.begin(),rowv.end());
    int last_robot = std::max(num_robots-2, 0);
    if(colv[last_task] < 0)
    {
        // no tasks remaining, assign this robot
        int dummy, winner;
        bid_mat_.col(col).maxCoeff(&winner, &dummy);
        cout << "winner is " << winner << " for task " << col << std::endl;
        processWinner(winner, col);
        cout << "selected winner, new bids are: " << endl;
        cout << bid_mat_ << std::endl;
    } else if (rowv[last_robot] < 0)
    {
        // no tasks remaining, assign this robot
        int dummy, winner;
        bid_mat_.row(row).maxCoeff(&winner, &dummy);
        cout << "winner is " << winner << " for task " << col << std::endl;
        processWinner(winner, col);
        cout << "selected winner, new bids are: " << endl;
        cout << bid_mat_ << std::endl;
    }
    else
    {
      bid_mat_(row, col) = -1;
    }

  }
    // selectWinner(winning_robot, winning_task);
    // cout << "winner is: " << winning_robot << ", " << winning_task << endl;
    // processWinner(winning_robot, winning_task);
    // calculateBids(winning_robot);
  // }
  cout << "Allocations: " << endl;
  printPaths();

  formOutput();
  return allocations;
}

/********************************************
 * Allocation processing (medium level)
 ********************************************/

// Clears paths, sets the correct size for bidding matrix.
void SequentialAuction::prepareAllocations()
{
  unalloc.clear();
  paths.clear();
  path_costs.clear();
  bids.clear();
  allocations.clear();
  bid_mat_ = Eigen::MatrixXd::Constant(num_robots, num_tasks,-1.0);

  for (int i = 0; i < num_tasks; i++)
  {
    unalloc.push_back(i);
  }
  for (int i = 0; i < num_robots; i++)
  {
    vector<int> v;
    paths.push_back(v);
    path_costs.push_back(0);
  }
  for (int i = 0; i < num_robots; i++)
  {
    vector<double> v;
    for (int j = 0; j < num_tasks; j++)
    {
      v.push_back(-1);
    }
    bids.push_back(v);
  }
}

void SequentialAuction::calculateAllBids()
{
  for (int i = 0; i < num_robots; i++)
  {
    calculateBids(i);
  }
}

// Populates the bidding matrix with bids for the currently unallocated tasks.
void SequentialAuction::calculateBids(int robot_num)
{
  for (int i = 0; i < unalloc.size(); i++)
  {
    int task_num = unalloc[i];
    if (!feasible_tasks.empty())
    {
      bool current_robot_can_do_current_task = feasible_tasks.at(robot_num).count(task_num);
      if (!current_robot_can_do_current_task)
      {
        ROS_DEBUG("Robot %i cannot do task %i; Bidding invalid amount.", robot_num, task_num);
        bids[robot_num][task_num] = -1;
        continue;
      }
    }

    // cout << "task " << unalloc[i] << endl;
    // Get the current path cost.
    double prev_cost = path_costs[robot_num];
    // Calculate the new path cost if we were to add the task to the path.
    vector<int> new_path;
    double new_cost = insertTask(robot_num, task_num, new_path);
    // Bid the new path cost.
    bids[robot_num][task_num] = new_cost;
    bid_mat_(robot_num,task_num) = new_cost;
  }
}

// Selects the winning robot and task by the minimum non-negative bid.
void SequentialAuction::selectWinner(int& winning_robot, int& winning_task)
{
  winning_robot = -1;
  winning_task = -1;

  // Option - winner is the least contested bid.
  if (use_least_contested_bid && num_robots > 1)
  {
    selectLeastContestedWinner(winning_robot, winning_task);
  } else {
    // Option (default) - winner is the lowest bid.
    double min_bid = -1;
    for (int i = 0; i < num_robots; i++)
    {
      for (int j = 0; j < num_tasks; j++)
      {
        double bid = bids[i][j];
        if ((bid < min_bid || min_bid == -1) && bid >= 0)
        {
          min_bid = bid;
          winning_robot = i;
          winning_task = j;
        }
      }
    }

    // int row, col;

    // while(true)
    // {
      

    // }

    //   for(auto i : bid_mat_.col(col))
    //   {
    //     std::sort()
    //   }
    //   if (bid_mat_.col(col).sum() == num_tasks*-1)
    //   {
    //     // we ran out of tasks, need to assign this one 

    //   }
    //   if (bid_mat_.row(row) == Eigen::Vectord(num_tasks, -1))
    //   {

    //   }

    // }
    // // we want the allocation that minimizes overall movement
    // for (int task=0; task<num_tasks; task++)
    // {
    //   for (int robot=0; robot<num_robots; robot++)
    //   {
    //     // 

    //   }

    // }

  }

  ROS_FATAL_COND(winning_robot == -1 || winning_task == -1, "No winner found.");
}

void SequentialAuction::selectLeastContestedWinner(int& winning_robot, int& winning_task)
{
  static constexpr double INF = std::numeric_limits<double>::infinity();

  ROS_ASSERT(winning_robot == -1);
  ROS_ASSERT(winning_task == -1);
  ROS_ASSERT(use_least_contested_bid);

  double winning_bid = INF;
  double winning_bid_diff = 0;

  for (int task : unalloc) {
    // For the current task, figure out which robot would win the task, with what bid, and by how much vs next lowest bid.
    int task_winning_robot = -1;
    double min_bid = INF;
    // The most contentious bid is the closest bid to the minimum bid; ie the second lowest bid.
    double most_contentious_bid = INF;

    // Look through the robots to find the one that wins this task.
    for (int robot = 0; robot < num_robots; robot++) {
      bool current_robot_can_do_current_task = feasible_tasks.empty() || feasible_tasks.at(robot).count(task);
      if (!current_robot_can_do_current_task) {
        continue;
      }

      double bid = bids[robot][task];
      if ( bid == -1 ) {
        continue;
      }

      ROS_ASSERT( bid >= 0 );
      ROS_ASSERT( bid < INF );

      if (bid < min_bid) {
        most_contentious_bid = min_bid;
        min_bid = bid;
        task_winning_robot = robot;
      } else if (bid < most_contentious_bid) {
        most_contentious_bid = bid;
      }
    }

    // Now, decide if this task is the new (temporary) winner.
    // 1. if current and winning tasks are both contested, pick the one with the least contention.
    //    ie, the biggest difference between the winning bid and the next lowest bid.
    // 2. if current and winning tasks are both uncontested, pick the one with the lowest bid.
    // 3. Otherwise, pick the uncontested task over the contested one.

    // 0. Reject the task if it has no valid bids.
    if ( min_bid == INF || task_winning_robot == -1 ) {
      ROS_WARN("Task %i has no valid bids.", task);
      continue;
    }

    double bid_diff = most_contentious_bid - min_bid; // INF for uncontested tasks.
    if (isTaskLessContentious(min_bid, bid_diff, winning_bid, winning_bid_diff)) {
      ROS_ASSERT(bid_diff >= winning_bid_diff);
      winning_robot = task_winning_robot;
      winning_task = task;
      winning_bid = min_bid;
      winning_bid_diff = bid_diff;
    }
  }
}

static bool isTaskLessContentious(double min_bid, double bid_diff, double winning_bid, double winning_bid_diff) {
  if (bid_diff < winning_bid_diff) {
    // Either the winning task is uncontested (infinite winning_bid_diff) and the current task is contested (finite bid_diff);
    // Or both tasks are contested, but the current task is more contested.
    // Either way, the current task loses.
    return false;
  }

  if (bid_diff == winning_bid_diff) {
    // Either both tasks are contested (finite bid_diff/winning_bid_diff) and equally contentious;
    // Or both tasks are uncontested (infinite bid_diff/winning_bid_diff).
    // Either way, the lowest bid wins.
    if (winning_bid <= min_bid) {
      // Current task doesn't have a lower bid and loses.
      return false;
    }
  }

  return true;
}

// Adds the winning task to the winning robots path, and removes the task from
// the unallocated list. Also sets all bids for the winning task to -1.
void SequentialAuction::processWinner(int winning_robot, int winning_task)
{
  insertTask(winning_robot, winning_task, paths[winning_robot]);
  if (!feasible_tasks.empty())
  {
    // For now, treat each robot (end effector) as being able to carry out a single task (waypoint trajectory).
    // As such, clear out its feasible tasks.
    // TODO: Generalize to something more sophisticated like
    // selectively removing only tasks which are mutually exclusive with the winning_task.
    ROS_DEBUG("Clearing feasible tasks for robot %i after winning task %i.", winning_robot, winning_task);
    //
    feasible_tasks.at(winning_robot).clear();
  }
  for (int i = 0; i < num_robots; i++)
  {
    bid_mat_(i, winning_task) = -1;
    bids[i][winning_task] = -1;
  }

  for(int i=0; i<num_tasks; i++)
  {
    bid_mat_(winning_robot, i) = -1;
  }

  for (int i = 0; i < unalloc.size(); i++)
  {
    if (unalloc[i] == winning_task)
    {
      unalloc.erase(unalloc.begin() + i);
      return;
    }
  }
}

// Populates the allocations variable.
void SequentialAuction::formOutput()
{
  allocations.clear();
  for (int i = 0; i < num_robots; i++)
  {
    TaskArray ta;
    vector<int> path = paths[i];
    for (int j = 0; j < path.size(); j++)
    {
      int id = path[j];
      ta.array.push_back(tasks[id]);
    }
    allocations.push_back(ta);
  }
}

/********************************************
 * (low level).
 ********************************************/

// Calculate the new path if a task were to be inserted into a path. Returns
// the cost of the new path.
double SequentialAuction::insertTask(int robot_num, int unalloc_id, vector<int>& new_path)
{
  vector<int> path = paths[robot_num];
  double path_cost;
  vector<int> best_path;
  double best_path_cost = -1;
  for (int i = 0; i <= path.size(); i++)
  {
    path.insert(path.begin() + i, unalloc_id);
    // printPath( path );
    path_cost = calculatePathCost(robot_num, path);
    // cout << "cost: " << path_cost << endl;
    if (path_cost < best_path_cost || best_path_cost == -1)
    {
      best_path_cost = path_cost;
      best_path = path;
    }
    path.erase(path.begin() + i);
  }

  new_path = best_path;
  return best_path_cost;
}

// Calculates the cost to go from the robots start pose to each task in order,
// then returning home (optionally).
double SequentialAuction::calculatePathCost(int robot_num, vector<int> path)
{
  double dist = 0;
  double x_prev, y_prev, z_prev, x_next, y_next, z_next, x_diff, y_diff, z_diff;
  int task_id;
  x_prev = robot_poses[robot_num].position.x;
  y_prev = robot_poses[robot_num].position.y;
  z_prev = robot_poses[robot_num].position.z;
  for (int i = 0; i < path.size(); i++)
  {
    task_id = path[i];
    x_next = tasks[task_id].pose.position.x;
    y_next = tasks[task_id].pose.position.y;
    z_next = tasks[task_id].pose.position.z;
    x_diff = x_next - x_prev;
    y_diff = y_next - y_prev;
    z_diff = z_next - z_prev;
    dist += std::hypot(x_diff, y_diff, z_diff);
    x_prev = x_next;
    y_prev = y_next;
    z_prev = z_next;
  }
  if (return_home)
  {
    x_next = robot_poses[robot_num].position.x;
    y_next = robot_poses[robot_num].position.y;
    z_next = robot_poses[robot_num].position.z;
    x_diff = x_next - x_prev;
    y_diff = y_next - y_prev;
    z_diff = z_next - z_prev;
    dist += std::hypot(x_diff, y_diff, z_diff);
  }
  return dist;
}

/********************************************
 * Printouts, purely for debugging.
 ********************************************/
void SequentialAuction::printPaths()
{
  for (int i = 0; i < num_robots; i++)
  {
    printPath(paths[i]);
  }
}

void SequentialAuction::printPath(vector<int> path)
{
  std::stringstream ss;
  ss << "[ ";
  for (int i = 0; i < path.size(); i++)
  {
    ss << path[i] << " ";
  }
  ss << "]";
  ROS_DEBUG_STREAM(ss.str());
}

void SequentialAuction::printBids()
{
  std::stringstream ss;
  ss << std::setprecision(1) << std::fixed;
  for (int i = 0; i < num_robots; i++)
  {
    for (int j = 0; j < num_tasks; j++)
    {
      ss << std::setw(10) << bids[i][j] << " ";
    }
    ss << endl;
  }
  ROS_DEBUG_STREAM(ss.str());
  cout << ss.str();
}
