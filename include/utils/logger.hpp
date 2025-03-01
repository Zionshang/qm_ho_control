#pragma once
#include <queue>
#include <fstream>
#include "Eigen/Dense"
#include "ctrl_component.hpp"

class Logger
{
public:
    Logger();
    void loadData();
    void saveData();

private:

    template <typename Scalar>
    void appendMat(const Eigen::DenseBase<Scalar> &mat);

    template <typename Scalar>
    void appendVar(const Scalar &var);
    std::string getTime();

    static constexpr int MAX_ROW = 60000;      // 最多保存的数据行数。60000=60秒，仿真时间尽量不要超过这个时间值，超过后前面的数据会被舍弃掉
    static constexpr int MAX_COL = 300;        // 一行最多能保存的数据个数
    int var_count;                             // 运行一次（1ms）保存的变量的数量，即csv文件中一行的数据数量
    int run_times;                             // 运行的次数，1ms一次
    std::deque<double> logger_buffer[MAX_COL]; // 队列容器，用来保存运行中的所有需要保存的数据
};

