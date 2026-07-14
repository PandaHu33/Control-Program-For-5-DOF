#include "mainpulator/motion_csv_logger.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const std::string& path) {
    std::ifstream file(path.c_str());
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

std::size_t countLinesStartingWith(const std::string& text, const std::string& prefix) {
    std::size_t count = 0;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.compare(0, prefix.size(), prefix) == 0) ++count;
    }
    return count;
}

std::size_t countLines(const std::string& text) {
    std::size_t count = 0;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) ++count;
    return count;
}

TEST(MotionCsvLogger, WritesHeaderDrainsAndAppendsWithoutSecondHeader) {
    const std::string path = "/tmp/mainpulator_motion_csv_logger_test.csv";
    std::remove(path.c_str());
    mainpulator_motion::MotionCsvLoggerConfig config;
    config.path = path;
    config.queue_capacity = 16;
    config.flush_rows = 100;

    mainpulator_motion::MotionLogSample sample;
    sample.active_source = "test,source";
    sample.planner_result = "working";
    sample.expected_q[0] = 0.25;
    {
        mainpulator_motion::MotionCsvLogger logger;
        ASSERT_TRUE(logger.start(config));
        ASSERT_TRUE(logger.push(sample));
        logger.stop();
    }
    {
        mainpulator_motion::MotionCsvLogger logger;
        ASSERT_TRUE(logger.start(config));
        ASSERT_TRUE(logger.push(sample));
        logger.stop();
    }

    const std::string contents = readFile(path);
    EXPECT_EQ(countLinesStartingWith(contents, "session_id,"), 1u);
    EXPECT_EQ(countLines(contents), 3u);
    EXPECT_NE(contents.find("raw_q1,raw_q2,raw_q3,raw_q4"), std::string::npos);
    EXPECT_NE(contents.find("expected_dq1,expected_dq2,expected_dq3,expected_dq4"), std::string::npos);
    EXPECT_NE(contents.find("actual_q1,actual_q2,actual_q3,actual_q4"), std::string::npos);
    EXPECT_NE(contents.find("\"test,source\""), std::string::npos);
    std::remove(path.c_str());
}

TEST(MotionCsvLogger, DisabledLoggerNeverAcceptsSamples) {
    mainpulator_motion::MotionCsvLogger logger;
    mainpulator_motion::MotionLogSample sample;
    EXPECT_FALSE(logger.push(sample));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
