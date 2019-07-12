//
// Created by yangcheng on 2019/7/4.
//

#ifndef LOCATION_XGBOOSTDETECTOR_H
#define LOCATION_XGBOOSTDETECTOR_H

#include "StopDetection.h"
#include <unordered_map>
#include <vector>
#include <memory>

struct XTree {
    int feature_idx_;
    double split_condition_;
    int left_node_;
    int right_node_;
    int miss_node_;
    double leaf_weight_;

    // tree node
    XTree(int feature_idx, double split_condition, int left_node, int right_node, int miss_node) :
            feature_idx_(feature_idx), split_condition_(split_condition), left_node_(left_node),
            right_node_(right_node), miss_node_(miss_node), leaf_weight_(0.0) {};

    // leaf node
    XTree(double weight) :
            feature_idx_(-1), split_condition_(0.0), left_node_(-1),
            right_node_(-1), miss_node_(-1), leaf_weight_(weight) {};

};

class XgboostDetector : public StopDetection {
private:

    double missing_feature = 999.0;
    typedef std::shared_ptr<XTree> XTree_ptr;
    typedef std::shared_ptr<std::unordered_map<int, XTree_ptr>> XTree_map_ptr;
    std::vector<XTree_map_ptr> XTrees;

    void decompress();

    XTree detectTrees(std::string &model_line);
    std::vector<double> predictTrees(std::vector<double> &current_input) const ;

public:

    explicit XgboostDetector(std::string &model_path);

    ~XgboostDetector();

    bool IsStopping(Eigen::VectorXd &data) const override;
};


#endif //LOCATION_XGBOOSTDETECTOR_H
