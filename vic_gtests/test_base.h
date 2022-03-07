#pragma once

#include "pch.h"

#include "vic/linalg/linalg.h"
#include "vic/linalg/inverse.h"
#include "vic/linalg/transpose.h"
#include "vic/linalg/determinant.h"
#include "vic/linalg/matrix_view.h"

using namespace vic::linalg;


template <typename TMat1, typename TMat2>
void ExpectMatrixEqual(const TMat1& mat1, const TMat2& mat2)
{
	ASSERT_EQ(TMat1::Rows, TMat2::Rows);
	ASSERT_EQ(TMat1::Columns, TMat2::Columns);

	for (std::size_t i = 0; i < TMat1::Rows; ++i)
		for (std::size_t j = 0; j < TMat1::Columns; ++j)
			EXPECT_DOUBLE_EQ(mat1.Get(i, j), mat2.Get(i, j));
}
