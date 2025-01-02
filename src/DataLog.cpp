#include "DataLog.h"

DataLog::DataLog()
{
    varCount = 0;
    runTimes = 0;
}

/**
 * @brief Load data during the run
 * @param est State Estimator
 * @param highCmd high command
 */
void DataLog::loadData(Estimator *est, HighCmd *highCmd)
{
    appendVar(est->getCurrentTime()); // 0

    appendMat(est->getPosCoM());          // 1,2,3
    appendMat(est->getVelCoM());          // 4,5,6
    appendMat(quat2RPY(est->getQuatB())); // 7,8,9
    appendMat(est->getAngVelB());         // 10,11,12

    appendMat(highCmd->posCoM);          // 13,14,15
    appendMat(highCmd->velCoM);          // 16,17,18
    appendMat(quat2RPY(highCmd->quatB)); // 19,20,21
    appendMat(highCmd->angVelB);         // 22,23,24

    appendMat(est->getQArm().head(3)); // 25,26,27
    appendMat(highCmd->qAJ.head(3));  // 28,29,30


    appendMat(est->getQArm().tail(3)); // 31,32,33
    appendMat(highCmd->qAJ.tail(3));  // 34,35,36
    varCount = 0;
    runTimes++;
}

/**
 * @brief save data after running
 */
void DataLog::saveData()
{
    std::string fileName = "../log/";
    std::ofstream fout;
    fileName += getTime();
    fileName += ".csv";
    fout.open(fileName, std::ios::out);

    if (runTimes > MAX_ROW)
        runTimes = MAX_ROW;

    for (int i = 0; i < runTimes; i++)
    {
        for (int j = 0; j < MAX_COL; j++)
        {
            if (!LogDataBuff[j].empty())
                fout << LogDataBuff[j].at(i) << ","; // 从数据缓存中输出数据到csv文件
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
std::string DataLog::getTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    return tmp;
}