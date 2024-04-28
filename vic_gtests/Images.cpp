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

    for(std::size_t j = 0; j < bitmap.Height(); ++j)
    {
        for(std::size_t i = 0; i < bitmap.Width(); ++i)
        {
            const bool f = ((i / 10) % 2) != ((j / 10) % 2);

            Color3 color{uchar(std::round(255. * f)), //
                         uchar(std::round(255. * f)),
                         uchar(std::round(255. * f))};

            bitmap.SetColor(i, j, color);
        }
    }

    const std::filesystem::path path = "SaveLoadBMP.bmp";
    EXPECT_EQ(SaveBMP(path, bitmap), ESaveStatus::OK);

    std::optional<Bitmap> optionalBitmap = LoadBMP(path);
    ASSERT_TRUE(optionalBitmap);

    ASSERT_TRUE(ImagesEqual(bitmap, *optionalBitmap));

    // todo: cleanup file?
}
