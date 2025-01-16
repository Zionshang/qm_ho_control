#pragma once
#include <queue>
#include <fstream>
#include "Estimator.h"
#include "HighCmd.h"
#include "Eigen/Dense"
#include "ctrl_component.hpp"

class DataLog
{
public:
    DataLog();
    void loadData(Estimator *est, HighCmd *highCmd);
    void saveData();

private:
    template <typename T>
    void appendMat(const Eigen::DenseBase<T> &mat);
    template <typename T>
    void appendVar(const T &var);
    std::string getTime();

    static constexpr int MAX_ROW = 60000;    // 最多保存的数据行数。60000=60秒，仿真时间尽量不要超过这个时间值，超过后前面的数据会被舍弃掉
    static constexpr int MAX_COL = 300;      // 一行最多能保存的数据个数
    int varCount;                            // 运行一次（1ms）保存的变量的数量，即csv文件中一行的数据数量
    int runTimes;                            // 运行的次数，1ms一次
    std::deque<double> LogDataBuff[MAX_COL]; // 队列容器，用来保存运行中的所有需要保存的数据
};

/**
 * @brief 添加矩阵数据
 * @param mat 添加的矩阵数据
 */
template <typename T>
void DataLog::appendMat(const Eigen::DenseBase<T> &mat)
{
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
        {
            LogDataBuff[varCount].push_back(mat(i, j));

            if (runTimes > MAX_ROW)
                LogDataBuff[varCount].pop_front();

            varCount++;
        }
}

/**
 * @brief 添加单个变量
 * @param var 添加的单个变量
 */
template <typename T>
void DataLog::appendVar(const T &var)
{
    LogDataBuff[varCount].push_back(var);

    if (runTimes > MAX_ROW)
        LogDataBuff[varCount].pop_front();

    varCount++;
}