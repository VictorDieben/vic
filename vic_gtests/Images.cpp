#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/image/image.h"

#include <cstdio>
#include <cstdlib>

using namespace vic::image;

template <typename TImage1, typename TImage2>
    requires ConceptImage<TImage1> && ConceptImage<TImage2>
bool ImagesEqual(const TImage1& image1, const TImage2& image2)
{
    if(image1.Width() != image2.Width() || //
       image1.Height() != image2.Height())
        return false;

    for(std::size_t j = 0; j < image1.Height(); ++j)
        for(std::size_t i = 0; i < image1.Width(); ++i)
            if(image1.GetColor(i, j) != image2.GetColor(i, j))
                return false;

    return true;
}

TEST(Images, SaveLoadBMP)
{
    Bitmap bitmap(123, 123);

    const std::size_t squareSize = 10;

    for(std::size_t j = 0; j < bitmap.Height(); ++j)
    {
        for(std::size_t i = 0; i < bitmap.Width(); ++i)
        {
            const bool f = ((i / squareSize) % 2) == ((j / squareSize) % 2);

            Color3 color{uchar(255 * f), //
                         uchar(255 * f),
                         uchar(255 * f)};

            bitmap.SetColor(i, j, color);
        }
    }

    const std::filesystem::path path = "SaveLoadBMP.bmp";
    ASSERT_EQ(SaveBMP(path, bitmap), ESaveStatus::OK);

    const std::optional<Bitmap> optionalBitmap = LoadBMP(path);
    ASSERT_TRUE(optionalBitmap);
    ASSERT_TRUE(ImagesEqual(bitmap, *optionalBitmap));

    // todo: cleanup file?
}
