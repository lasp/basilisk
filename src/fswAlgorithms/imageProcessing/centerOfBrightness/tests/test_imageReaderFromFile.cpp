// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "../imageReader/imageReaderFromFile.h"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _WIN32
#include <process.h>
static inline int currentProcessId() { return _getpid(); }
#else
#include <unistd.h>
static inline int currentProcessId() { return getpid(); }
#endif

#include <cstdio>
#include <filesystem>
#include <memory>
#include <set>

// ============================================================================
// FIXTURE
// ============================================================================

class ImageReaderFromFileTest : public ::testing::Test {
   protected:
    ImageReaderFromFile reader;
    std::string tempDir;
    // Heap-allocated to avoid stack overflow (~8 MB array)
    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> outputStorage =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    std::array<Eigen::Vector2i, kMaxWindowSize>& output = *outputStorage;

    void SetUp() override {
        // Use a temp directory unique per test (and per process).
        const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
        tempDir = std::filesystem::temp_directory_path().string() + "/imageReaderFileTest_" + info->test_suite_name() +
                  "_" + info->name() + "_" + std::to_string(currentProcessId());
        std::filesystem::remove_all(tempDir);  // clear any stale leftover
        std::filesystem::create_directories(tempDir);
    }

    void TearDown() override { std::filesystem::remove_all(tempDir); }

    // Create a BGR image: black background with white (255,255,255) pixels at given locations
    std::string createTestImage(int width, int height, const std::vector<Eigen::Vector2i>& whitePixels) {
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
        for (const auto& p : whitePixels) {
            // OpenCV: mat.at<Vec3b>(row, col) = mat.at<Vec3b>(y, x)
            img.at<cv::Vec3b>(p[1], p[0]) = cv::Vec3b(255, 255, 255);
        }
        std::string path = tempDir + "/test_image.png";
        cv::imwrite(path, img);
        return path;
    }

    // Create a BGR image with specific gray values at given pixel locations
    std::string createGrayValueImage(int width,
                                     int height,
                                     const std::vector<std::pair<Eigen::Vector2i, uint8_t>>& pixels) {
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
        for (const auto& [p, val] : pixels) {
            img.at<cv::Vec3b>(p[1], p[0]) = cv::Vec3b(val, val, val);
        }
        std::string path = tempDir + "/gray_test_image.png";
        cv::imwrite(path, img);
        return path;
    }

    // Collect all non-zero (non-sentinel) pixel coordinates from output array
    std::set<std::pair<int, int>> collectNonZeroPixels(const std::array<Eigen::Vector2i, kMaxWindowSize>& output) {
        std::set<std::pair<int, int>> result;
        for (const auto& coord : output) {
            if (coord[0] != 0 || coord[1] != 0) {
                result.insert({coord[0], coord[1]});
            }
        }
        return result;
    }
};

// ============================================================================
// SETTER / GETTER TESTS
// ============================================================================

TEST_F(ImageReaderFromFileTest, SettersGetters) {
    reader.setBlurSize(5);
    EXPECT_EQ(5, reader.getBlurSize());

    reader.setPixelThreshold(128.0);
    EXPECT_DOUBLE_EQ(128.0, reader.getPixelThreshold());

    reader.setFileName("/some/path.png");
    EXPECT_EQ("/some/path.png", reader.getFileName());

    reader.setSaveImages(true);
    EXPECT_TRUE(reader.getSaveImages());

    reader.setSaveDir("/tmp/save");
    EXPECT_EQ("/tmp/save", reader.getSaveDir());
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

TEST_F(ImageReaderFromFileTest, ThrowsOnEmptyFilename) {
    Eigen::Vector2i center(50, 50);
    Eigen::Vector2i windowSize(100, 100);
    EXPECT_THROW(reader.getImageAsArray(center, windowSize, output), std::invalid_argument);
}

TEST_F(ImageReaderFromFileTest, WindowSizeOverflow) {
    std::string path = createTestImage(20, 20, {});
    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    Eigen::Vector2i center(500, 500);
    // 1025 * 1025 > kMaxWindowSize (1024*1024)
    Eigen::Vector2i windowSize(1025, 1025);
    EXPECT_THROW(reader.getImageAsArray(center, windowSize, output), std::invalid_argument);
}

// ============================================================================
// IMAGE SIZE
// ============================================================================

TEST_F(ImageReaderFromFileTest, GetFullImageSize) {
    std::string path = createTestImage(30, 20, {});
    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    // Need to call getImageAsArray first to trigger readImageFromFile

    reader.getImageAsArray(Eigen::Vector2i(15, 10), Eigen::Vector2i(30, 20), output);

    Eigen::Vector2i size = reader.getFullImageSize(0);
    EXPECT_EQ(30, size[0]);
    EXPECT_EQ(20, size[1]);
}

// ============================================================================
// CHECK FOR NEW IMAGE
// ============================================================================

TEST_F(ImageReaderFromFileTest, CheckForNewImageAlwaysReturnsOne) {
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(0, 0));
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(0, 999));
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(42, 12345));
}

// ============================================================================
// NON-ZERO PIXEL LOCATIONS
// ============================================================================

TEST_F(ImageReaderFromFileTest, NonZeroPixelLocations) {
    // Create 20x20 black image with 3 known white pixels
    std::vector<Eigen::Vector2i> whitePixels = {
        Eigen::Vector2i(5, 5), Eigen::Vector2i(10, 10), Eigen::Vector2i(15, 15)};
    std::string path = createTestImage(20, 20, whitePixels);

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    // Full-image window
    reader.getImageAsArray(Eigen::Vector2i(10, 10), Eigen::Vector2i(20, 20), output);

    auto found = collectNonZeroPixels(output);

    EXPECT_EQ(3u, found.size());
    EXPECT_TRUE(found.count({5, 5}));
    EXPECT_TRUE(found.count({10, 10}));
    EXPECT_TRUE(found.count({15, 15}));
}

// ============================================================================
// WINDOW FILTERING
// ============================================================================

TEST_F(ImageReaderFromFileTest, WindowFiltering) {
    // White pixels at (3,3), (10,10), (17,17) in a 20x20 image
    std::vector<Eigen::Vector2i> whitePixels = {
        Eigen::Vector2i(3, 3), Eigen::Vector2i(10, 10), Eigen::Vector2i(17, 17)};
    std::string path = createTestImage(20, 20, whitePixels);

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    // Window: center=(5,5), size=(10,10) → covers [0,10) x [0,10)
    reader.getImageAsArray(Eigen::Vector2i(5, 5), Eigen::Vector2i(10, 10), output);

    auto found = collectNonZeroPixels(output);

    // Only (3,3) should be in the window [0,10) x [0,10)
    EXPECT_TRUE(found.count({3, 3}));
    EXPECT_FALSE(found.count({10, 10}));
    EXPECT_FALSE(found.count({17, 17}));
}

TEST_F(ImageReaderFromFileTest, WindowExcludesOutside) {
    // White pixels spread across image
    std::vector<Eigen::Vector2i> whitePixels = {Eigen::Vector2i(2, 2), Eigen::Vector2i(18, 18)};
    std::string path = createTestImage(20, 20, whitePixels);

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    // Window centered at (15,15) size (8,8) → covers [11,19) x [11,19)
    reader.getImageAsArray(Eigen::Vector2i(15, 15), Eigen::Vector2i(8, 8), output);

    auto found = collectNonZeroPixels(output);

    // Only (18,18) is inside [11,19)
    EXPECT_TRUE(found.count({18, 18}));
    EXPECT_FALSE(found.count({2, 2}));
}

// ============================================================================
// THRESHOLD FILTERING
// ============================================================================

TEST_F(ImageReaderFromFileTest, ThresholdFiltering) {
    // Pixels with gray value 100 and 200
    std::string path = createGrayValueImage(20, 20, {{Eigen::Vector2i(5, 5), 100}, {Eigen::Vector2i(15, 15), 200}});

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(150);

    reader.getImageAsArray(Eigen::Vector2i(10, 10), Eigen::Vector2i(20, 20), output);

    auto found = collectNonZeroPixels(output);

    // Only the pixel with value 200 should pass threshold 150
    EXPECT_TRUE(found.count({15, 15}));
    EXPECT_FALSE(found.count({5, 5}));
}

// ============================================================================
// BLUR EFFECT
// ============================================================================

TEST_F(ImageReaderFromFileTest, BlurEffect) {
    // Single bright pixel — with blur=1 (no blur), only that pixel should appear
    std::string path = createTestImage(20, 20, {Eigen::Vector2i(10, 10)});

    // blur=1: no blurring, should find exactly 1 pixel
    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    reader.getImageAsArray(Eigen::Vector2i(10, 10), Eigen::Vector2i(20, 20), output);
    auto foundNoBlur = collectNonZeroPixels(output);

    // blur=5: blurring spreads the pixel, more neighbors pass threshold
    reader.setBlurSize(5);
    reader.getImageAsArray(Eigen::Vector2i(10, 10), Eigen::Vector2i(20, 20), output);
    auto foundWithBlur = collectNonZeroPixels(output);

    EXPECT_EQ(1u, foundNoBlur.size());
    // With blur, more pixels should pass (or fewer if blur dilutes below threshold)
    // The key test: the results should differ
    EXPECT_NE(foundNoBlur.size(), foundWithBlur.size());
}

// ============================================================================
// ZERO SENTINEL
// ============================================================================

TEST_F(ImageReaderFromFileTest, ZeroSentinel) {
    // Image with exactly 1 white pixel
    std::string path = createTestImage(20, 20, {Eigen::Vector2i(10, 10)});

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    reader.getImageAsArray(Eigen::Vector2i(10, 10), Eigen::Vector2i(20, 20), output);

    // First entry should be the pixel
    EXPECT_EQ(Eigen::Vector2i(10, 10), output[0]);

    // Remaining entries should be zero sentinel
    for (std::size_t i = 1; i < 100; ++i) {
        EXPECT_EQ(Eigen::Vector2i::Zero(), output[i]);
    }
}
