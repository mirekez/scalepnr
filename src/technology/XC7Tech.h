#pragma once

#include "Technology.h"
#include "Design.h"

struct XC7Tech: public Technology
{
    void init();

    bool estimateTimings(rtl::Design& design)
    {
        for(auto& module : design.modules) {
            for (auto& cell : module.cells) {
                if (cell.type == "LUT6") {
                    cell.latency_matrix.resize(6);
                    for (int i=0; i<6; ++i) {
                        cell.latency_matrix.push_back(0.05);
                    }
                    continue;
                }
                if (cell.type == "LUT5") {
                    cell.latency_matrix.resize(5);
                    for (int i=0; i<5; ++i) {
                        cell.latency_matrix.push_back(0.05);
                    }
                    continue;
                }
                if (cell.type == "LUT4") {
                    cell.latency_matrix.resize(4);
                    for (int i=0; i<4; ++i) {
                        cell.latency_matrix.push_back(0.05);
                    }
                    continue;
                }
                if (cell.type == "LUT3") {
                    cell.latency_matrix.resize(3);
                    for (int i=0; i<3; ++i) {
                        cell.latency_matrix.push_back(0.05);
                    }
                    continue;
                }
                if (cell.type == "LUT2") {
                    cell.latency_matrix.resize(2);
                    for (int i=0; i<2; ++i) {
                        cell.latency_matrix.push_back(0.05);
                    }
                    continue;
                }
                PNR_ERROR("unknown cell type: {}", cell.type);
                return false;
            }
        }
        return true;
    }


    static XC7Tech& current();

};
