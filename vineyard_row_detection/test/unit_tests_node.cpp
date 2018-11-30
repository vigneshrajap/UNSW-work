
#include <gtest/gtest.h>

#include <vineyard_row_detection/ransac_line_detection.h>

TEST(Line, lineCoefficentsConstructor)
{

  Eigen::VectorXd line_coefficients = Eigen::VectorXd::Zero(6);
  line_coefficients << 1.0,
                       0.0,
                       0.0,
                       0.0,
                       1.0,
                       0.0;

  vineyard_row_detection::Line line(line_coefficients);
  line.makeRadiusPositive();

  EXPECT_FLOAT_EQ(line.radius(), 1.0);
  EXPECT_FLOAT_EQ(line.angle(), 0.0);

  line_coefficients <<  1.0,
                        0.0,
                        0.0,
                       -1.0,
                        1.0,
                        0.0;

  line = vineyard_row_detection::Line(line_coefficients);
  line.makeRadiusPositive();

  EXPECT_FLOAT_EQ(line.radius(), 1.0/sqrt(2.0));
  EXPECT_FLOAT_EQ(line.angle(), M_PI_4);

  line.shiftOrigin(0.0, 1.0);

  EXPECT_FLOAT_EQ(line.radius(), 0.0);
  EXPECT_FLOAT_EQ(line.angle(), M_PI_4);

}

TEST(RANSACLineDetection, removePointsFromCloud)
{

  vineyard_row_detection::point_cloud_ptr_t cloud(new vineyard_row_detection::point_cloud_t());

  for (int i = 0; i < 10; ++i)
  {
    vineyard_row_detection::point_t point;
    point.x = (double)i;
    point.y = 0.0;
    point.z = 0.0;
    cloud->points.push_back(point);
  }

  std::vector<int> inliers;
  inliers.push_back(0);

  vineyard_row_detection::removePointsFromCloud(*cloud, inliers);

  EXPECT_EQ(cloud->points.size(), 9);
  for (int i = 0; i < cloud->points.size(); ++i)
  {
    EXPECT_FLOAT_EQ(cloud->points.at(i).x, (float)(i + 1));
  }

}

int main(int argc, char ** argv)
{

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}


