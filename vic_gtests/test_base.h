#pragma once

#include "pch.h"

#include "vic/linalg/determinant.h"
#include "vic/linalg/inverse.h"
#include "vic/linalg/linalg.h"
#include "vic/linalg/matrix_view.h"
#include "vic/linalg/transpose.h"

using namespace vic::linalg;

template <typename TMat1, typename TMat2>
void ExpectMatrixEqual(const TMat1& mat1, const TMat2& mat2)
{
    ASSERT_EQ(mat1.GetRows(), mat2.GetRows());
    ASSERT_EQ(mat1.GetColumns(), mat2.GetColumns());

    for(std::size_t i = 0; i < mat1.GetRows(); ++i)
        for(std::size_t j = 0; j < mat2.GetColumns(); ++j)
            EXPECT_DOUBLE_EQ(mat1.Get(i, j), mat2.Get(i, j));
}
