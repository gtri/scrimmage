/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef SimpleCaptureMetrics_H_
#define SimpleCaptureMetrics_H_
#include <iostream>
#include <scrimmage/metrics/Metrics.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

class Score {
 public:
    Score() {}

    bool set_weights(std::map<std::string,std::string> &params)
    {
        // TODO: Automate this weight parsing
        double w = sc::get<double>("TeamCapture_weight", params, 0.0);
        weights_["TeamCapture"] = w;
        
        w = sc::get<double>("NonTeamCapture_weight", params, 0.0);
        weights_["NonTeamCapture"] = w;
        return true;
    }

    void increment_count(std::string type)
    {
        auto it = counts_.find(type);
        if (it == counts_.end()) {
            counts_[type] = 1;
        } else {
            (it->second)++;
        }
    }

    void add_count(std::string type, int c)
    {
        auto it = counts_.find(type);
        if (it == counts_.end()) {
            counts_[type] = c;
        } else {
            (it->second) += c;
        }
    }

    void set_count(std::string type, int c)
    {
        counts_[type] = c;
    }

    int count(std::string type)
    {
        auto it = counts_.find(type);
        if (it == counts_.end()) {
            return 0;
        }
        return it->second;
    }

    double score()
    {        
        // Apply weights to all scores
        double s = 0;
        for (auto &kv : counts_) {
            auto it = weights_.find(kv.first);
            if (it != weights_.end()) {
                s += kv.second * it->second;
            } else {
                cout << "Metrics Warning: Count exists without weight" << endl;
                cout << "Count Name: " << kv.first << endl;
            }
        }
        return s;
    }

protected:
    std::map<std::string,int> counts_;
    std::map<std::string,double> weights_;
};


class SimpleCaptureMetrics : public scrimmage::Metrics {
public:
    SimpleCaptureMetrics();
    virtual std::string name() { return std::string("SimpleCaptureMetrics"); }
    virtual void init(std::map<std::string,std::string> &params);
    virtual bool step_metrics(double t, double dt);
    virtual void calc_team_scores();
    virtual void print_team_summaries();
protected:
    sc::SubscriberPtr team_capture_sub_;
    sc::SubscriberPtr nonteam_capture_sub_;

    std::map<int, Score> scores_;
    std::map<int, Score> team_scores_map_;

    std::map<std::string,std::string> params_;
private:
};

#endif
