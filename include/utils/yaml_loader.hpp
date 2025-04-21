#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <vector>

/**
 * 结构体用于存储YAML配置文件中的参数
 */
struct YamlParams
{
    // 权重矩阵
    Eigen::VectorXd W_ee_diag;    
    Eigen::VectorXd W_vel_diag;  
    Eigen::VectorXd W_u_diag;     
    Eigen::VectorXd w_x_body_pos; 
    Eigen::VectorXd w_x_arm_pos;  
    Eigen::VectorXd w_x_body_vel; 
    Eigen::VectorXd w_x_arm_vel;  
    YamlParams(const std::string &filepath)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(filepath);

            // 读取权重矩阵
            W_ee_diag = yamlSequenceToEigen(config["W_ee_diag"]);
            W_vel_diag = yamlSequenceToEigen(config["W_vel_diag"]);
            W_u_diag = yamlSequenceToEigen(config["W_u_diag"]);
            w_x_body_pos = yamlSequenceToEigen(config["w_x_body_pos"]);
            w_x_arm_pos = yamlSequenceToEigen(config["w_x_arm_pos"]);
            w_x_body_vel = yamlSequenceToEigen(config["w_x_body_vel"]);
            w_x_arm_vel = yamlSequenceToEigen(config["w_x_arm_vel"]);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        }
    }

    /**
     * 将YAML序列转换为Eigen::VectorXd
     * @param node YAML序列节点
     * @return 包含序列值的Eigen向量
     */
    Eigen::VectorXd yamlSequenceToEigen(const YAML::Node &node)
    {
        if (!node.IsSequence())
        {
            throw std::runtime_error("YAML node is not a sequence");
        }

        Eigen::VectorXd vec(node.size());
        for (size_t i = 0; i < node.size(); ++i)
        {
            vec(i) = node[i].as<double>();
        }
        return vec;
    }
};