#include "gtest/gtest.h"

#include "test_base.h"

#include "vic/sound/sound.h"
#include "vic/utils/file.h"

TEST(Sound, ReadWave)
{
    // todo: put sample wav somewhere in this project
    const auto filepath = std::filesystem::path("C:/Users/victo/source/repos/EntSim/resources/sounds/test_sample.wav");
    const auto file = vic::FileToByteVec(filepath);
    ASSERT_TRUE(file.has_value());

    auto waveBytes = file.value();
    const auto span = std::span<std::byte>{waveBytes.data(), waveBytes.size()};

    const auto wave = vic::sound::ReadWave(span);

    int bla = 1;
}