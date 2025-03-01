#include "utils/logger.hpp"

Logger::Logger()
{
    var_count = 0;
    run_times = 0;
}

/**
 * @brief 添加矩阵数据
 * @param mat 添加的矩阵数据
 */
template <typename Scalar>
void Logger::appendMat(const Eigen::DenseBase<Scalar> &mat)
{
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
        {
            logger_buffer[var_count].push_back(mat(i, j));

            if (run_times > MAX_ROW)
                logger_buffer[var_count].pop_front();

            var_count++;
        }
}

/**
 * @brief 添加单个变量
 * @param var 添加的单个变量
 */
template <typename Scalar>
void Logger::appendVar(const Scalar &var)
{
    logger_buffer[var_count].push_back(var);

    if (run_times > MAX_ROW)
        logger_buffer[var_count].pop_front();

    var_count++;
}

/**
 * @brief Load data during the run
 * @param est State Estimator
 * @param highCmd high command
 */
void Logger::loadData()
{
    // appendVar(est->getCurrentTime()); // 0

    // appendMat(est->getPosCoM());          // 1,2,3
    // appendMat(est->getVelCoM());          // 4,5,6
    // appendMat(quat2RPY(est->getQuatB())); // 7,8,9
    // appendMat(est->getAngVelB());         // 10,11,12

    // appendMat(highCmd->posCoM);          // 13,14,15
    // appendMat(highCmd->velCoM);          // 16,17,18
    // appendMat(quat2RPY(highCmd->quatB)); // 19,20,21
    // appendMat(highCmd->angVelB);         // 22,23,24

    // appendMat(est->getQArm().head(3)); // 25,26,27
    // appendMat(highCmd->qAJ.head(3));  // 28,29,30

    // appendMat(est->getQArm().tail(3)); // 31,32,33
    // appendMat(highCmd->qAJ.tail(3));  // 34,35,36
    // var_count = 0;
    // run_times++;
}

/**
 * @brief save data after running
 */
void Logger::saveData()
{
    std::string fileName = "../log/";
    std::ofstream fout;
    fileName += getTime();
    fileName += ".csv";
    fout.open(fileName, std::ios::out);

    if (run_times > MAX_ROW)
        run_times = MAX_ROW;

    for (int i = 0; i < run_times; i++)
    {
        for (int j = 0; j < MAX_COL; j++)
        {
            if (!logger_buffer[j].empty())
                fout << logger_buffer[j].at(i) << ","; // 从数据缓存中输出数据到csv文件
        }
        fout << std::endl;
    }
    std::cout << "[Save log] file writed" << std::endl;
    fout.close();
    std::cout << "[Save log] " << fileName << std::endl;
}

/**
 * @brief get current system time, used as file name
 */
std::string Logger::getTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    return tmp;
}