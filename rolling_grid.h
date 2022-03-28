

/**
 * @file rolling_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling 3D grid
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include <grid/grid.h>
#include <utils/misc_utils.h>

    namespace rolling_grid_ns
{
    class RollingGrid
    {
    public:
        explicit RollingGrid(const Eigen::Vector3i& size);
        ~RollingGrid() = default;
        bool InRange(Eigen::Vector3i sub) const
        {
            return grid0_->InRange(sub);
        }
        bool InRange(int ind) const//判断索引号是否在范围内
        {
            return grid0_->InRange(ind);
        }
        Eigen::Vector3i Ind2Sub(int ind) const//根据索引号找到对应的sub，如索引号是7，那么就找到第7个sub
        {
            return grid0_->Ind2Sub(ind);
        }
        int Sub2Ind(Eigen::Vector3i sub) const//根据sub找到索引，即根据sub的坐标找到是第几个sub
        {
            return grid0_->Sub2Ind(sub);
        }
        int GetArrayInd(Eigen::Vector3i sub) const
        {
            MY_ASSERT(InRange(sub));
            if (which_grid_)
            {
                return grid1_->GetCellValue(sub);//根据sub坐标找到索引值ind，然后返回cells_[ind];由此可见，sub只是一个子空间，它的坐标是某个cell的顶点坐标，真正存值的是cell
            }
            else
            {
                return grid0_->GetCellValue(sub);
            }
        }
        int GetArrayInd(int ind) const
        {
            MY_ASSERT(InRange(ind));
            Eigen::Vector3i sub = grid0_->Ind2Sub(ind);
            return GetArrayInd(sub);
        }
        int GetInd(int array_ind) const
        {
            MY_ASSERT(InRange(array_ind));
            return array_ind_to_ind_[array_ind];
        }

        void Roll(const Eigen::Vector3i& roll_dir);
        void GetUpdatedIndices(std::vector<int>& updated_indices) const;
        void GetRolledOutIndices(const Eigen::Vector3i& roll_dir, std::vector<int>& rolled_out_indices);
        void GetUpdatedArrayIndices(std::vector<int>& updated_array_indices) const;

    private:
        Eigen::Vector3i size_;
        std::unique_ptr<grid_ns::Grid<int>> grid0_;
        std::unique_ptr<grid_ns::Grid<int>> grid1_;
        std::vector<int> updated_indices_;
        std::vector<int> array_ind_to_ind_;
        bool which_grid_;

        inline int GetFromIdx(int cur_idx, int roll_step, int max_idx) const
        {
            return cur_idx <= roll_step - 1 ? max_idx - roll_step + cur_idx : cur_idx - roll_step;
        }
        void RollHelper(const std::unique_ptr<grid_ns::Grid<int>>& grid_in,
            const std::unique_ptr<grid_ns::Grid<int>>& grid_out, Eigen::Vector3i roll_dir);

        void GetRolledInIndices(const Eigen::Vector3i& roll_dir);
        void GetRolledOutIndices(const Eigen::Vector3i& roll_dir);
        void GetIndices(std::vector<int>& indices, Eigen::Vector3i start_idx, Eigen::Vector3i end_idx) const;
    };
}  // namespace rolling_grid_ns

