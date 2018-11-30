
#include <gtest/gtest.h>

#include <vineyard_navigation/row_waypoint_cost_function.h>

TEST(RowWaypointCostFunction, parseRowWaypointsAndOrder)
{

  std::vector<std::vector<double> > row_waypoints;
  std::vector<int> row_order;

  std::vector<double> waypoint(6);

  waypoint.at(0) = 0.0;
  waypoint.at(1) = 0.0;
  waypoint.at(3) = 0.0;
  waypoint.at(4) = 1.0;
  row_waypoints.push_back(waypoint);

  waypoint.at(0) = 1.0;
  waypoint.at(1) = 0.1;
  waypoint.at(3) = 1.0;
  waypoint.at(4) = 1.1;
  row_waypoints.push_back(waypoint);

  waypoint.at(0) = 2.0;
  waypoint.at(1) = 0.2;
  waypoint.at(3) = 2.0;
  waypoint.at(4) = 1.2;
  row_waypoints.push_back(waypoint);

  waypoint.at(0) = 3.0;
  waypoint.at(1) = 0.3;
  waypoint.at(3) = 3.0;
  waypoint.at(4) = 1.3;
  row_waypoints.push_back(waypoint);

  row_order.push_back(0);
  row_order.push_back(1);
  row_order.push_back(2);
  row_order.push_back(3);
  row_order.push_back(0);

  vineyard_navigation::RowWaypointCostFunction cost_func_1;

  cost_func_1.parseRowWaypointsAndOrder(row_waypoints, row_order, 0);

  EXPECT_EQ(cost_func_1.getWaypoints().size(), 12);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(0).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(0).at(1), 0.0);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(1).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(1).at(1), 1.0);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(2).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(2).at(1), 1.1);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(3).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(3).at(1), 0.1);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(4).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(4).at(1), 0.2);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(5).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(5).at(1), 1.2);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(6).at(0), 3.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(6).at(1), 1.3);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(7).at(0), 3.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(7).at(1), 0.3);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(8).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(8).at(1), 0.2);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(9).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(9).at(1), 0.1);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(10).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(10).at(1), 0.0);

  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(11).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_1.getWaypoints().at(11).at(1), 1.0);

  vineyard_navigation::RowWaypointCostFunction cost_func_2;
  cost_func_2.parseRowWaypointsAndOrder(row_waypoints, row_order, 1);

  EXPECT_EQ(cost_func_2.getWaypoints().size(), 12);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(0).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(0).at(1), 1.0);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(1).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(1).at(1), 0.0);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(2).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(2).at(1), 0.1);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(3).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(3).at(1), 1.1);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(4).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(4).at(1), 1.2);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(5).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(5).at(1), 0.2);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(6).at(0), 3.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(6).at(1), 0.3);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(7).at(0), 3.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(7).at(1), 1.3);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(8).at(0), 2.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(8).at(1), 1.2);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(9).at(0), 1.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(9).at(1), 1.1);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(10).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(10).at(1), 1.0);

  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(11).at(0), 0.0);
  EXPECT_FLOAT_EQ(cost_func_2.getWaypoints().at(11).at(1), 0.0);

}

int main(int argc, char ** argv)
{

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}


