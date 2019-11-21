/*
 * testing.cpp
 *
 *  Created on: 13/07/2015
 *      Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 *
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2015, Francesco Verdoja
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its 
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "supervoxel_clustering/testing.h"

/**
 * Initialize values of all performance scores, used in the constructor
 */
void Testing::init_performance() {
    precision = -1;
    recall = -1;
    fscore = -1;
    voi = -1;
    wov = -1;
    fpr = -1;
    fnr = -1;
}

/**
 * Converts a segmented pointcloud into a map of segments
 * 
 * @param in    the pointcloud to be converted
 * 
 * @return a map where each segment can be accessed by its label
 */
labelMapT Testing::label_map(PointLCloudT::Ptr in) {
    labelMapT map_temp, map;
    PointLCloudT::iterator it = in->begin();
    for (; it != in->end(); ++it) {
        uint32_t l = it->label;
        if (map_temp.count(l) == 0) {
            PointLCloudT::Ptr subcloud = extract_label_cloud(in, l);
            map_temp.insert(
                    std::pair<uint32_t, PointLCloudT::Ptr>(l, subcloud));
        }
    }

    uint32_t new_l = 0;
    labelMapT::iterator it_m = map_temp.begin();
    for (; it_m != map_temp.end(); ++it_m) {
        map.insert(std::pair<uint32_t, PointLCloudT::Ptr>(new_l, it_m->second));
        new_l++;
    }

    return map;
}

/**
 * Compute the intersection between the segmentation to be tested and the 
 * groundtruth and store the best matches in an internal object state
 */
void Testing::compute_intersections() {
    uint32_t n = segm_labels.size();
    uint32_t m = truth_labels.size();
    inter_matrix = Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(
            n, m);
    matches = Eigen::Array<int64_t, 1, Eigen::Dynamic>::Zero(1, m) - 1;

    std::map<size_t, uint32_t> t_sizes;

    labelMapT::iterator it_s = segm_labels.begin();

    uint32_t i = 0;
    for (; it_s != segm_labels.end(); ++it_s) {
        uint32_t j = 0;
        labelMapT::iterator it_t = truth_labels.begin();

        for (; it_t != truth_labels.end(); ++it_t) {
            t_sizes.insert(
                    std::pair<size_t, uint32_t>(it_t->second->size(), j));
            size_t inter = count_intersect(it_s->second, it_t->second);
            inter_matrix(i, j) = inter;
            j++;
        }
        i++;
    }

    std::map<size_t, uint32_t>::reverse_iterator it_ts = t_sizes.rbegin();
    for (; it_ts != t_sizes.rend(); ++it_ts) {
        Eigen::Matrix<size_t, Eigen::Dynamic, 1> col = inter_matrix.col(
                it_ts->second);
        int64_t row;
        size_t max = col.maxCoeff(&row);
        pcl::console::print_debug("Testing best match: %d - %d - %d\t",
                it_ts->second, max, row);
        while ((matches == row).any()) {
            col(row) = 0;
            if (col.array().any()) {
                size_t max = col.maxCoeff(&row);
                pcl::console::print_debug("NO\nTesting best match: %d - %d - %d\t",
                        it_ts->second, max, row);
            } else {
                row = -1;
                pcl::console::print_debug("NO\nBest match not found: %d\t",
                        it_ts->second);
                break;
            }
        }
        pcl::console::print_debug("OK\n");
        matches(it_ts->second) = row;
    }

    /*
     * TODO 
     * The following lines should use pcl::console::print_debug, disabled for the
     * moment. Can be uncommented while debugging this class.
     */
    //std::cout << "Intersection matrix:\n" << inter_matrix << "\n";
    //std::cout << "Best matches:\n" << matches << "\n";
}

/**
 * Extract all points corresponding to a region from the segmentation
 * @param c     the segmentation
 * @param label the label of the region to be extracted
 * 
 * @return a pointcloud containing all points corresponding to the given region
 */
PointLCloudT::Ptr Testing::extract_label_cloud(PointLCloudT::Ptr c,
        uint32_t label) const {
    PointLCloudT::Ptr subcloud = boost::make_shared<PointLCloudT>();
    PointLCloudT::iterator it = c->begin();
    for (; it != c->end(); ++it) {
        if (it->label == label)
            subcloud->push_back(*it);
    }

    return subcloud;
}

/**
 * Compute the number of points in the intersection between two regions
 * 
 * @param c1    the first region
 * @param c2    the second region
 * 
 * @return the cardinality of the intersection between the two given regions
 */
size_t Testing::count_intersect(PointLCloudT::Ptr c1,
        PointLCloudT::Ptr c2) const {
    pcl::console::print_debug("Searching intersection: %d - %d\n", c1->size(),
            c2->size());

    compareXYZ cmp;

    PointLVectorT c1s = c1->points;
    PointLVectorT c2s = c2->points;

    std::sort(c1s.begin(), c1s.end(), cmp);
    std::sort(c2s.begin(), c2s.end(), cmp);

    PointLVectorT res;

    std::set_intersection(c1s.begin(), c1s.end(), c2s.begin(), c2s.end(),
            std::back_inserter(res), cmp);
    return res.size();
}

/**
 * Compute the number of points in the union of two regions
 * 
 * @param c1    the first region
 * @param c2    the second region
 * 
 * @return the cardinality of the union of the two given regions
 */
size_t Testing::count_union(PointLCloudT::Ptr c1, PointLCloudT::Ptr c2) const {
    pcl::console::print_debug("Searching union: %d - %d\n", c1->size(), c2->size());

    compareXYZ cmp;

    PointLVectorT c1s = c1->points;
    PointLVectorT c2s = c2->points;

    std::sort(c1s.begin(), c1s.end(), cmp);
    std::sort(c2s.begin(), c2s.end(), cmp);

    PointLVectorT res;

    std::set_union(c1s.begin(), c1s.end(), c2s.begin(), c2s.end(),
            std::back_inserter(res), cmp);
    return res.size();
}

/**
 * Constructor for the Testing class
 * 
 * @param s a segmentation of the pointcloud to be tested
 * @param t the corresponding segmentation groundtruth
 */
Testing::Testing(PointLCloudT::Ptr s, PointLCloudT::Ptr t) {
    is_set_segm = false;
    is_set_truth = false;
    set_segm(s);
    set_truth(t);
}

/**
 * Compute the precision score
 * 
 * @return the score
 */
float Testing::eval_precision() {
    if (precision == -1) {
        float p = 0;
        float r = 0;
        float fp = 0;
        float fn = 0;
        for (uint32_t j = 0; j < truth_labels.size(); j++) {
            int64_t i = matches(j);
            if (i != -1) {
                float inter = inter_matrix(i, j);
                float s = (segm_labels.at(i))->size();
                float g = (truth_labels.at(j))->size();
                p += inter * g / s;
                r += inter;
                fp += (s - inter);
                fn += (g - inter);
            } else {
                float g = (truth_labels.at(j))->size();
                fn += g;
            }
        }
        float N = truth->size();
        precision = p / N;
        recall = r / N;
        fpr = fp / N;
        fnr = fn / N;
    }

    return precision;
}

/**
 * Compute the recall score
 * 
 * @return the score
 */
float Testing::eval_recall() {
    if (recall == -1) {
        eval_precision();
    }

    return recall;
}

/**
 * Compute the F-score
 * 
 * @return the score
 */
float Testing::eval_fscore() {
    if (precision == -1)
        eval_precision();

    if (precision == 0 && recall == 0) {
        pcl::console::print_warn(
                "Both precision and recall equal to 0; setting f-score to 0 to avoid denominator to be equal to 0.");
        fscore = 0;
    } else
        fscore = 2 * (precision * recall) / (precision + recall);

    return fscore;
}

/**
 * Compute the Variation of Information (VoI) score
 * 
 * @return the score
 */
float Testing::eval_voi() {
    if (voi == -1) {
        float h_s = 0;
        float h_t = 0;
        float mi = 0;
        float n = truth->size();

        int i = 0;
        labelMapT::iterator it_s = segm_labels.begin();
        for (; it_s != segm_labels.end(); ++it_s) {
            float p = it_s->second->size();
            h_s -= std::log(p / n) * p / n;

            int j = 0;
            labelMapT::iterator it_t = truth_labels.begin();
            for (; it_t != truth_labels.end(); ++it_t) {
                float q = it_t->second->size();
                if (i == 0)
                    h_t -= std::log(q / n) * q / n;
                float r = inter_matrix(i, j);
                if (r != 0) {
                    mi += std::log(((n * r) / (p * q))) * r / n;
                }
                j++;
            }
            i++;
        }
        voi = h_s + h_t - 2 * mi;
    }

    return voi;
}

/**
 * Compute the weighted Overlap (wOv) score
 * 
 * @return the score
 */
float Testing::eval_wov() {
    if (wov == -1) {
        float w = 0;
        for (uint32_t j = 0; j < truth_labels.size(); j++) {
            int64_t i = matches(j);
            if (i != -1) {
                float inter = inter_matrix(i, j);
                float un = count_union(segm_labels.at(i), truth_labels.at(j));
                float g = (truth_labels.at(j))->size();
                w += inter * g / un;
            }
        }
        float N = truth->size();
        wov = w / N;
    }

    return wov;
}

/**
 * Compute the False Positive Rate (FPR) score
 * 
 * @return the score
 */
float Testing::eval_fpr() {
    if (fpr == -1) {
        eval_precision();
    }

    return fpr;
}

/**
 * Compute the False Negative Rate (FNR) score
 * 
 * @return the score
 */
float Testing::eval_fnr() {
    if (fnr == -1) {
        eval_precision();
    }

    return fnr;
}

/**
 * Compute scores for all test metrics
 * 
 * @return a struct containing all scores
 */
performanceSet Testing::eval_performance() {
    performanceSet perf;
    perf.voi = eval_voi();
    perf.precision = eval_precision();
    perf.recall = recall;
    perf.fscore = eval_fscore();
    perf.wov = eval_wov();
    perf.fpr = fpr;
    perf.fnr = fnr;

    return perf;
}

/**
 * Set the segmentation to be tested
 * 
 * @param s the new segmentation to be tested
 */
void Testing::set_segm(PointLCloudT::Ptr s) {
    if (s->empty())
        throw std::invalid_argument(
            "The pointcloud to be set as 'segm' cannot be empty");
    segm = s;
    init_performance();
    segm_labels = label_map(s);
    is_set_segm = true;
    if (is_set_truth)
        compute_intersections();
}

/**
 * Set the new groundtruth
 * 
 * @param t a new segmentation to be used as groundtruth
 */
void Testing::set_truth(PointLCloudT::Ptr t) {
    if (t->empty())
        throw std::invalid_argument(
            "The pointcloud to be set as 'truth' cannot be empty");
    truth = t;
    init_performance();
    truth_labels = label_map(t);
    is_set_truth = true;
    if (is_set_segm)
        compute_intersections();
}
