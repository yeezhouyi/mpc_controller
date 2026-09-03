// C1: reference validation seams (dimension / NaN / freshness).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "mpc_controller/reference_validation.hpp"

namespace mpc_controller
{

TEST(ReferenceValidation, CorrectDimensionPasses)
{
  EXPECT_TRUE(validReferenceDimension(std::vector<double>{0.1, 0.2, 0.3, 0.4}, 4));
}

TEST(ReferenceValidation, WrongDimensionRejected)
{
  EXPECT_FALSE(validReferenceDimension(std::vector<double>{0.1, 0.2, 0.3}, 4));
  EXPECT_FALSE(validReferenceDimension(std::vector<double>{}, 4));
}

TEST(ReferenceValidation, NanReferenceRejected)
{
  EXPECT_FALSE(referenceAllFinite(std::vector<double>{0.0, std::nan(""), 1.0}));
  EXPECT_FALSE(referenceAllFinite(std::vector<double>{0.0, std::numeric_limits<double>::infinity()}));
}

TEST(ReferenceValidation, FiniteReferenceAccepted)
{
  EXPECT_TRUE(referenceAllFinite(std::vector<double>{0.0, -0.5, 1e-9}));
}

TEST(ReferenceValidation, FreshnessWindow)
{
  EXPECT_TRUE(isReferenceFresh(0.0, 5.0));
  EXPECT_TRUE(isReferenceFresh(4.999, 5.0));
  EXPECT_FALSE(isReferenceFresh(5.001, 5.0));  // stale
  EXPECT_FALSE(isReferenceFresh(-1.0, 5.0));   // future stamp -> not trusted
}

}  // namespace mpc_controller
