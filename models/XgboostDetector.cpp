//
// Created by yangcheng on 2019/7/4.
//

#include "XgboostDetector.h"
#include "../utils/Tools.h"
#include <fstream>
#include <iostream>

XgboostDetector::~XgboostDetector() {};

/**
 * xgboost read model and decompress into shared_ptr<XTree>.
 * @param model_path
 */
XgboostDetector::XgboostDetector(std::string &model_path) {
    Tools tools;
    XTree_map_ptr xtree_map_ptr = std::make_shared<std::unordered_map<int, XTree_ptr>>();

    std::ifstream infile(model_path);
    std::string line;
    std::getline(infile, line);

    int boost_id = 0;
    while (std::getline(infile, line)) {
        if (line.find("booster") != std::string::npos) {
            boost_id += 1;
            this->XTrees.push_back(xtree_map_ptr);
            xtree_map_ptr.reset(new std::unordered_map<int, XTree_ptr>());
        } else {
            int node_id = std::stoi(tools.split(line, ":")[0]);
            XTree xtree = this->detectTrees(line);
            XTree_ptr xtree_ptr = std::make_shared<XTree>(xtree);
//            std::cout.precision(10);
//            std::cout << "booster id " << boost_id
//                      << " node id " << node_id
//                      << " feature idx: " << xtree.feature_idx_
//                      << " condition: " <<  xtree.split_condition_
//                      << " yes node " << xtree.left_node_
//                      << " no node " << xtree.right_node_
//                      << " miss node " << xtree.miss_node_
//                      << " leaf weight " << xtree.leaf_weight_
//                      << std::endl;
            xtree_map_ptr->emplace(std::make_pair(node_id, xtree_ptr));
        }
    }
    this->XTrees.push_back(xtree_map_ptr);

}

/**
 * detect tree node and convert it into XTree structure.
 *
 * @param model_line: file read line.
 * @return XTree
 */
XTree XgboostDetector::detectTrees(std::string &model_line) {

    Tools tools;
    if (model_line.find("leaf") == std::string::npos) {

        // e.g "0:[f5<0.999779344] yes=1,no=2,missing=1" => "0:[f5<0.999779344]" "yes=1,no=2,missing=1"
        std::vector<std::string> feature_node = tools.split(model_line, " ");
        // e.g "0:[f5<0.999779344]" => "0:[f5" "0.999779344]"
        std::vector<std::string> feature_s = tools.split(feature_node[0], "<");

        // feature idx; e.g "0:[f5" => "0:" "f5"
        std::string feature_idx_s = tools.split(feature_s[0], "[")[1];
        feature_idx_s.erase(0, 1);
        int feature_idx = std::stoi(feature_idx_s);

        // split condition; e.g "0.999779344]" => 0.999779344
        std::string split_condition_s = feature_s[1];
        split_condition_s.pop_back();
        double split_condition = std::stod(split_condition_s);

        // yes/no/missing
        std::vector<std::string> node_s = tools.split(feature_node[1], ",");
        int yes_node = std::stoi(tools.split(node_s[0], "=")[1]);
        int no_node = std::stoi(tools.split(node_s[1], "=")[1]);
        int missing_node = std::stoi(tools.split(node_s[2], "=")[1]);
        return {feature_idx, split_condition, yes_node, no_node, missing_node};

    } else {

        double leaf_value = std::stof(tools.split(model_line, "=")[1]);
        return {leaf_value};

    }

}

/**
 * predict the vector belong to which class.
 * Note: typedef std::shared_ptr<XTree> XTree_ptr;
 *       typedef std::shared_ptr<std::unordered_map<int, XTree_ptr>> XTree_map_ptr;
 *       std::vector<XTree_map_ptr> XTrees;
 *
 * @param current_input
 * @return
 */
std::vector<double> XgboostDetector::predictTrees(std::vector<double> &current_input) const {

    std::vector<double> res{0.0, 0.0};
    int tag = 0;
    for (auto &tree_map : this->XTrees) {

        int class_idx = tag % 2;
        tag += 1;

        int node_id = 0;
        while (tree_map->find(node_id) != tree_map->end()) {

            XTree_ptr tree = (*tree_map)[node_id];

            if (tree->feature_idx_ == -1) {
                res[class_idx] += tree->leaf_weight_;
                break;
            } else {
                double feature = current_input[tree->feature_idx_];
                if (feature != this->missing_feature) {
                    if (feature < tree->split_condition_) {
                        node_id = tree->left_node_;
                    } else {
                        node_id = tree->right_node_;
                    }
                } else {
                    node_id = tree->miss_node_;
                }
            }
        }
    }
    return res;
}


/**
 * predict current data using the xgboost model.
 *
 * @param data, input vector.
 * @return preidct probability vector for each class according to input training class order.
 */
bool XgboostDetector::IsStopping(Eigen::VectorXd &data) const {
    std::vector<double> inputs;
    size_t nums = data.rows();
    for (std::size_t i = 0; i < nums; ++i) {
        inputs.push_back(data(i));
    }
    std::vector<double> res = this->predictTrees(inputs);

    double stop_exp = exp(res[0]);
    double move_exp = exp(res[1]);
    double stop_prob = stop_exp / (stop_exp + move_exp);
    double move_prob = move_exp / (stop_exp + move_exp);
    return stop_prob > move_prob;
}

