#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/image/image.h"

#include <cstdio>
#include <cstdlib>

namespace vic
{
namespace image
{
//
//bool ImagesEqual(const Image& image1, const Image& image2)
//{
//    if(image1.Width() != image2.Width() || //
//       image1.Height() != image2.Height())
//        return false;
//
//    for(std::size_t j = 0; j < image1.Height(); ++j)
//        for(std::size_t i = 0; i < image1.Width(); ++i)
//            if(image1.Get(i, j) != image2.Get(i, j))
//                return false;
//    return true;
//}
//
//TEST(TestImages, SaveLoadBMP)
//{
//    Image image{300, 200};
//    for(std::size_t j = 0; j < image.Height(); ++j)
//    {
//        for(std::size_t i = 0; i < image.Width(); ++i)
//        {
//            double r = double(i) / (image.Width() - 1.);
//            double g = double(j) / (image.Height() - 1.);
//            double b = 0.;
//            Color color{uchar(std::round(255. * r)), //
//                        uchar(std::round(255. * g)),
//                        uchar(std::round(255. * b)),
//                        255};
//            image.Set(i, j, color);
//        }
//    }
//
//    const std::string path = "artifacts/SaveLoadBMP.bmp";
//    SaveBMP(path, image);
//    Image loadedImage = LoadBMP<Image>(path);
//
//    ASSERT_TRUE(ImagesEqual(image, loadedImage));
//}

} // namespace image
} // namespace vic